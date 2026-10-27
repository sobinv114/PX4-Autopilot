// /****************************************************************************
//  *
//  *   Copyright (c) 2024-2026 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  * 3. Neither the name PX4 nor the names of its contributors may be
//  *    used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/


/**
 * @file px4io_serial.cpp
 *
 * Serial interface for PX4IO on ESP32-P4, using the NuttX tty driver.
 */


#include <px4_arch/px4io_serial.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <stddef.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <px4_platform_common/log.h>
#include <board_config.h>
#include <sys/types.h>
#include <signal.h>

#ifndef PX4IO_SERIAL_DEVICE
# error "Board must define PX4IO_SERIAL_DEVICE (e.g. \"/dev/ttyS1\")"
#endif



#define PX4IO_EXCHANGE_TIMEOUT_NS	(20LL * 1000 * 1000)	/* 10 ms, same as STM32 20 ms for esp32p4*/

uint8_t ArchPX4IOSerial::_io_buffer_storage[ArchPX4IOSerial::kBufSize];

static size_t s_rx_total;	/* DEBUG: bytes received in the current exchange */

/* ---- time helpers ------------------------------------------------------ */

static int64_t timespec_to_ns(const struct timespec &ts)
{
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* milliseconds left until the deadline, rounded up, never negative */
static int ms_left(const struct timespec &deadline)
{
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);

	int64_t diff_ns = timespec_to_ns(deadline) - timespec_to_ns(now);

	if (diff_ns <= 0) {
		return 0;
	}

	return (int)((diff_ns + 999999LL) / 1000000LL);
}

/* ---- construction ------------------------------------------------------ */

ArchPX4IOSerial::ArchPX4IOSerial()
{
}

ArchPX4IOSerial::~ArchPX4IOSerial()
{
	/* only close an fd that belongs to the calling task group */
	if (_fd >= 0 && _fd_owner == getpid()) {
		::close(_fd);
	}

	_fd = -1;
}

int ArchPX4IOSerial::init()
{
	int r = PX4IO_serial::init((IOPacket *)&_io_buffer_storage[0]);

	if (r != 0) {
		return r;
	}

	return _open_port();
}

int ArchPX4IOSerial::_open_port()
{
	int fd = ::open(PX4IO_SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		return -errno;
	}

	struct termios t;

	if (tcgetattr(fd, &t) != 0) {
		int err = errno;
		::close(fd);
		return -err;
	}

	t.c_iflag = 0;
	t.c_oflag = 0;
	t.c_lflag = 0;
	t.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
#ifdef CRTS_IFLOW
	t.c_cflag &= ~CRTS_IFLOW;
#endif
#ifdef CCTS_OFLOW
	t.c_cflag &= ~CCTS_OFLOW;
#endif
	t.c_cflag |= (CS8 | CREAD | CLOCAL);
	cfsetspeed(&t, PX4IO_SERIAL_BITRATE);

	if (tcsetattr(fd, TCSANOW, &t) != 0) {
		int err = errno;
		::close(fd);
		return -err;
	}

	_fd = fd;
	_fd_owner = getpid();
	PX4_INFO("px4io uart opened: fd=%d owner=%d baud=%u", fd, (int)_fd_owner, (unsigned)PX4IO_SERIAL_BITRATE);
	return 0;
}


int ArchPX4IOSerial::ioctl(unsigned operation, unsigned &arg)
{
	(void)operation;
	(void)arg;
	return -1;
}

/* ---- low level helpers ------------------------------------------------- */

void ArchPX4IOSerial::_drain_rx()
{
	uint8_t scratch[32];

	for (int i = 0; i < 16; i++) {
		ssize_t n = ::read(_xfd, scratch, sizeof(scratch));

		if (n <= 0) {
			break;
		}
	}
}

int ArchPX4IOSerial::_write_all(const uint8_t *buf, size_t len, const struct timespec &deadline)
{
	size_t sent = 0;

	while (sent < len) {
		ssize_t n = ::write(_xfd, buf + sent, len - sent);

		if (n > 0) {
			sent += n;
			continue;
		}

		if (n < 0 && errno != EINTR && errno != EAGAIN) {
			PX4_ERR("_write_all failed: errno=%d (%s)", errno, strerror(errno));
			return -EIO;
		}

		struct pollfd pfd;
		pfd.fd = _xfd;
		pfd.events = POLLOUT;
		pfd.revents = 0;

		if (::poll(&pfd, 1, ms_left(deadline)) <= 0) {
			return -ETIMEDOUT;
		}
	}

	return OK;
}

int ArchPX4IOSerial::_read_exact(uint8_t *buf, size_t len, const struct timespec &deadline)
{
	size_t got = 0;

	while (got < len) {
		struct pollfd pfd;
		pfd.fd = _xfd;
		pfd.events = POLLIN;
		pfd.revents = 0;

		int pr = ::poll(&pfd, 1, ms_left(deadline));

		if (pr < 0) {
			if (errno == EINTR) {
				continue;
			}
			PX4_ERR("_read_exact poll failed: errno=%d (%s)", errno, strerror(errno));
			return -EIO;
		}

		if (pr > 0 && (pfd.revents & (POLLERR | POLLHUP))) {
			PX4_ERR("_read_exact revents error: 0x%x (POLLERR=%d, POLLHUP=%d)",
				(unsigned)pfd.revents, !!(pfd.revents & POLLERR), !!(pfd.revents & POLLHUP));
			return -EIO;
		}

		if (pr == 0) {
			return -ETIMEDOUT;
		}

		ssize_t n = ::read(_xfd, buf + got, len - got);

		if (n > 0) {
			got += n;
			s_rx_total += n;
			continue;
		}

		if (n < 0 && errno != EINTR && errno != EAGAIN) {
			PX4_ERR("_read_exact read failed: errno=%d (%s)", errno, strerror(errno));
			return -EIO;
		}
	}

	return OK;
}

/* ---- the transaction --------------------------------------------------- */
int ArchPX4IOSerial::_acquire_fd(bool &is_temp)
{
	is_temp = false;
	const pid_t me = getpid();

	/* 1. If we already own the main fd, keep using it */
	if (_fd >= 0 && _fd_owner == me) {
		return _fd;
	}

	/* 2. If another active task (e.g. the work queue) owns the main fd,
	 * use a temporary short-lived descriptor for this command (e.g. px4io status) */
	if (_fd >= 0 && kill(_fd_owner, 0) == 0) {
		int fd = ::open(PX4IO_SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (fd < 0) {
			return -errno;
		}

		is_temp = true;
		return fd;
	}

	/* 3. The old owner task exited (e.g. px4io start): become the new persistent owner */
	_fd = -1;
	int r = _open_port();
	return (r == 0) ? _fd : r;
}

int ArchPX4IOSerial::_bus_exchange(IOPacket *_packet)
{
	bool is_temp = false;
	int fd = _acquire_fd(is_temp);

	if (fd < 0) {
		return fd;
	}

	_xfd = fd;
	int ret = _do_exchange(_packet);
	_xfd = -1;

	if (is_temp) {
		::close(fd);
	}

	return ret;
}

int ArchPX4IOSerial::_do_exchange(IOPacket *_packet)
{
	perf_begin(_pc_txns);

	/* drop anything left over from a previous failed exchange */
	_drain_rx();
	s_rx_total = 0;

	/* absolute deadline on the monotonic clock */
	struct timespec deadline;
	clock_gettime(CLOCK_MONOTONIC, &deadline);
	int64_t dl_ns = timespec_to_ns(deadline) + PX4IO_EXCHANGE_TIMEOUT_NS;
	deadline.tv_sec  = dl_ns / 1000000000LL;
	deadline.tv_nsec = dl_ns % 1000000000LL;

	/* transmit */
	const size_t tx_len = PKT_SIZE(*_packet);
	int ret = _write_all((const uint8_t *)_packet, tx_len, deadline);

	/* receive: 4 byte header first, then the registers */
	if (ret == OK) {
		uint8_t *rx = (uint8_t *)_packet;
		const size_t hdr = offsetof(IOPacket, regs);

		ret = _read_exact(rx, hdr, deadline);

		if (ret == OK) {
			const unsigned count = PKT_COUNT(*_packet);

			if (count > PKT_MAX_REGS) {
				perf_count(_pc_badidle);
				ret = -EIO;

			} else if (count > 0) {
				ret = _read_exact(rx + hdr, count * sizeof(uint16_t), deadline);
			}
		}
	}

	if (ret != OK) {
		if (ret == -ETIMEDOUT) {
			perf_count(_pc_timeouts);
		} else {
			perf_count(_pc_uerrs);
		}

		static unsigned dbg_n = 0;
		if (dbg_n < 10) {
			dbg_n++;
			const uint8_t *b = (const uint8_t *)_packet;
			PX4_WARN("xchg fail ret=%d tx_len=%u rx_bytes=%u buf=%02x %02x %02x %02x",
				 ret, (unsigned)tx_len, (unsigned)s_rx_total, b[0], b[1], b[2], b[3]);
		}

		perf_cancel(_pc_txns);
		return ret;
	}

	/* check packet CRC */
	uint8_t crc = _packet->crc;
	_packet->crc = 0;

	if ((crc != crc_packet(_packet)) || (PKT_CODE(*_packet) == PKT_CODE_CORRUPT)) {
		perf_count(_pc_crcerrs);
		ret = -EIO;
	}

	perf_end(_pc_txns);
	return ret;
}
