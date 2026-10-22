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

// /**
//  * @file px4io_serial.cpp
//  *
//  * Serial interface for PX4IO on ESP32-P4 (NuttX POSIX UART).
//  */

// #include <px4_arch/px4io_serial.h>

// #include <fcntl.h>
// #include <termios.h>
// #include <unistd.h>
// #include <errno.h>
// #include <string.h>
// #include <sys/select.h>
// #include <sys/time.h>

// #include <px4_platform_common/log.h>
// #include <board_config.h>

// #ifndef PX4IO_SERIAL_DEVICE
// # error "Board must define PX4IO_SERIAL_DEVICE (e.g. \"/dev/ttyS1\")"
// #endif

// #ifndef PX4IO_SERIAL_BAUD
// # define PX4IO_SERIAL_BAUD 1500000
// #endif

// ArchPX4IOSerial::ArchPX4IOSerial()
// {
// }

// ArchPX4IOSerial::~ArchPX4IOSerial()
// {
// 	if (_fd >= 0) {
// 		::close(_fd);
// 		_fd = -1;
// 	}
// }

// int
// ArchPX4IOSerial::init()
// {
// 	/* Hand packet buffer to the base class (creates bus semaphore). */
// 	int r = PX4IO_serial::init(&_io_buffer);

// 	if (r != 0) {
// 		return r;
// 	}

// 	_fd = ::open(PX4IO_SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);

// 	if (_fd < 0) {
// 		PX4_ERR("PX4IO open %s failed: %d", PX4IO_SERIAL_DEVICE, errno);
// 		return -errno;
// 	}

// 	struct termios opt;

// 	if (tcgetattr(_fd, &opt) < 0) {
// 		::close(_fd);
// 		_fd = -1;
// 		return -errno;
// 	}

// 	cfmakeraw(&opt);
// 	cfsetispeed(&opt, PX4IO_SERIAL_BAUD);
// 	cfsetospeed(&opt, PX4IO_SERIAL_BAUD);

// 	opt.c_cflag |= (CLOCAL | CREAD);
// 	opt.c_cflag &= ~(CRTSCTS | CSIZE | PARENB);
// 	opt.c_cflag |= CS8;
// 	opt.c_cc[VMIN]  = 0;
// 	opt.c_cc[VTIME] = 0;	/* timeouts via select() */

// 	if (tcsetattr(_fd, TCSANOW, &opt) < 0) {
// 		::close(_fd);
// 		_fd = -1;
// 		return -errno;
// 	}

// 	tcflush(_fd, TCIOFLUSH);

// 	PX4_INFO("PX4IO serial on %s @ %d baud", PX4IO_SERIAL_DEVICE, PX4IO_SERIAL_BAUD);
// 	return OK;
// }

// int
// ArchPX4IOSerial::ioctl(unsigned operation, unsigned &arg)
// {
// 	(void)operation;
// 	(void)arg;
// 	return -ENOTTY;
// }

// int
// ArchPX4IOSerial::_bus_exchange(IOPacket *_packet)
// {
// 	if (_fd < 0 || _packet == nullptr) {
// 		return -EINVAL;
// 	}

// 	const size_t tx_len = PKT_SIZE(*_packet);

// 	/* ---------- TX ---------- */
// 	{
// 		const uint8_t *tx = reinterpret_cast<const uint8_t *>(_packet);
// 		size_t written = 0;

// 		while (written < tx_len) {
// 			ssize_t n = ::write(_fd, tx + written, tx_len - written);

// 			if (n > 0) {
// 				written += (size_t)n;
// 				continue;
// 			}

// 			if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
// 				fd_set wfds;
// 				FD_ZERO(&wfds);
// 				FD_SET(_fd, &wfds);
// 				struct timeval tv = { 0, 2000 }; /* 2 ms */

// 				if (select(_fd + 1, nullptr, &wfds, nullptr, &tv) <= 0) {
// 					perf_count(_pc_timeouts);
// 					return -ETIMEDOUT;
// 				}

// 				continue;
// 			}

// 			perf_count(_pc_uerrs);
// 			return -errno;
// 		}
// 	}

// 	/* ---------- RX ---------- */
// 	{
// 		uint8_t *rx = reinterpret_cast<uint8_t *>(_packet);
// 		size_t received = 0;
// 		size_t expect = sizeof(IOPacket); /* refined after header */

// 		while (received < expect) {
// 			fd_set rfds;
// 			FD_ZERO(&rfds);
// 			FD_SET(_fd, &rfds);
// 			struct timeval tv;
// 			tv.tv_sec  = 0;
// 			tv.tv_usec = EXCHANGE_TIMEOUT_MS * 1000;

// 			int sel = select(_fd + 1, &rfds, nullptr, nullptr, &tv);

// 			if (sel == 0) {
// 				perf_count(_pc_timeouts);
// 				return -ETIMEDOUT;
// 			}

// 			if (sel < 0) {
// 				perf_count(_pc_uerrs);
// 				return -errno;
// 			}

// 			ssize_t n = ::read(_fd, rx + received, expect - received);

// 			if (n > 0) {
// 				received += (size_t)n;

// 				/* After header (4 bytes) we know real packet length. */
// 				if (received >= 4) {
// 					size_t pkt = PKT_SIZE(*_packet);

// 					if (pkt < 4 || pkt > sizeof(IOPacket)) {
// 						perf_count(_pc_protoerrs);
// 						return -EIO;
// 					}

// 					expect = pkt;
// 				}

// 				continue;
// 			}

// 			if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
// 				continue;
// 			}

// 			if (n == 0) {
// 				continue;
// 			}

// 			perf_count(_pc_uerrs);
// 			return -errno;
// 		}
// 	}

// 	/* CRC is checked by the base class after we return OK. */
// 	return OK;
// }


























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

#ifndef PX4IO_SERIAL_DEVICE
# error "Board must define PX4IO_SERIAL_DEVICE (e.g. \"/dev/ttyS1\")"
#endif



#define PX4IO_EXCHANGE_TIMEOUT_NS	(10LL * 1000 * 1000)	/* 10 ms, same as STM32 */

uint8_t ArchPX4IOSerial::_io_buffer_storage[ArchPX4IOSerial::kBufSize];

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
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

int ArchPX4IOSerial::init()
{
	/* initialize base implementation */
	int r = PX4IO_serial::init((IOPacket *)&_io_buffer_storage[0]);

	if (r != 0) {
		return r;
	}

	/* O_NONBLOCK so that poll() is the only thing that controls timeouts */
	_fd = ::open(PX4IO_SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		return -errno;
	}

	struct termios t;

	if (tcgetattr(_fd, &t) != 0) {
		int err = errno;
		::close(_fd);
		_fd = -1;
		return -err;
	}

	/* raw mode */
	t.c_iflag = 0;
	t.c_oflag = 0;
	t.c_lflag = 0;

	/* 8N1, receiver on, ignore modem lines, no flow control */
	t.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
#ifdef CRTS_IFLOW
	t.c_cflag &= ~CRTS_IFLOW;
#endif
#ifdef CCTS_OFLOW
	t.c_cflag &= ~CCTS_OFLOW;
#endif
	t.c_cflag |= (CS8 | CREAD | CLOCAL);

	cfsetspeed(&t, PX4IO_SERIAL_BITRATE);

	if (tcsetattr(_fd, TCSANOW, &t) != 0) {
		int err = errno;
		::close(_fd);
		_fd = -1;
		return -err;
	}

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

	/* bounded, so a babbling line can't hold us here forever */
	for (int i = 0; i < 16; i++) {
		ssize_t n = ::read(_fd, scratch, sizeof(scratch));

		if (n <= 0) {
			break;
		}
	}
}

int ArchPX4IOSerial::_write_all(const uint8_t *buf, size_t len, const struct timespec &deadline)
{
	size_t sent = 0;

	while (sent < len) {
		ssize_t n = ::write(_fd, buf + sent, len - sent);

		if (n > 0) {
			sent += n;
			continue;
		}

		if (n < 0 && errno != EINTR && errno != EAGAIN) {
			return -EIO;
		}

		/* TX queue full: wait for space, but only until the deadline */
		struct pollfd pfd;
		pfd.fd = _fd;
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
		pfd.fd = _fd;
		pfd.events = POLLIN;
		pfd.revents = 0;

		int pr = ::poll(&pfd, 1, ms_left(deadline));

		if (pr < 0) {
			if (errno == EINTR) {
				continue;
			}

			return -EIO;
		}

		if (pr == 0) {
			return -ETIMEDOUT;
		}

		ssize_t n = ::read(_fd, buf + got, len - got);

		if (n > 0) {
			got += n;
			continue;
		}

		if (n < 0 && errno != EINTR && errno != EAGAIN) {
			return -EIO;
		}

		/* n == 0 or transient error: loop and poll again until the deadline */
	}

	return OK;
}

/* ---- the transaction --------------------------------------------------- */

int ArchPX4IOSerial::_bus_exchange(IOPacket *_packet)
{
	_current_packet = _packet;

	if (_fd < 0) {
		return -EBADF;
	}

	perf_begin(_pc_txns);

	/* drop anything left over from a previous failed exchange */
	_drain_rx();

	/* absolute 10 ms deadline on the monotonic clock */
	struct timespec deadline;
	clock_gettime(CLOCK_MONOTONIC, &deadline);
	int64_t dl_ns = timespec_to_ns(deadline) + PX4IO_EXCHANGE_TIMEOUT_NS;
	deadline.tv_sec  = dl_ns / 1000000000LL;
	deadline.tv_nsec = dl_ns % 1000000000LL;

	/* transmit */
	const size_t tx_len = PKT_SIZE(*_current_packet);
	int ret = _write_all((const uint8_t *)_current_packet, tx_len, deadline);

	/* receive: 4 byte header first (crc, count_code, page, offset), then the registers */
	if (ret == OK) {
		uint8_t *rx = (uint8_t *)_current_packet;
		const size_t hdr = offsetof(IOPacket, regs);

		ret = _read_exact(rx, hdr, deadline);

		if (ret == OK) {
			const unsigned count = PKT_COUNT(*_current_packet);

			if (count > PKT_MAX_REGS) {
				perf_count(_pc_badidle);
				ret = -EIO;

			} else if (count > 0) {
				ret = _read_exact(rx + hdr, count * sizeof(uint16_t), deadline);
			}
		}
	}

	/* timeouts and I/O errors: not a completed transaction */
	if (ret != OK) {
		if (ret == -ETIMEDOUT) {
			perf_count(_pc_timeouts);

		} else {
			perf_count(_pc_uerrs);
		}

		perf_cancel(_pc_txns);
		return ret;
	}

	/* check packet CRC - a corrupt packet from IO means IO saw a CRC error */
	uint8_t crc = _current_packet->crc;
	_current_packet->crc = 0;

	if ((crc != crc_packet(_current_packet)) || (PKT_CODE(*_current_packet) == PKT_CODE_CORRUPT)) {
		perf_count(_pc_crcerrs);
		ret = -EIO;
	}

	/* as upstream: a CRC failure still took real wire time, so it is recorded */
	perf_end(_pc_txns);
	return ret;
}
