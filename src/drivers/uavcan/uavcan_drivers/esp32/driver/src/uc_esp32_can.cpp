#ifndef MODULE_NAME
#define MODULE_NAME "uavcan_esp32"
#endif

#include <px4_platform_common/log.h>
#include <fcntl.h>
#include <poll.h>
#include <nuttx/can/can.h>
#include <uavcan_esp32/can.hpp>
#include <uavcan_esp32/clock.hpp>
#include <cstring>
#include <unistd.h>
#include <errno.h>

namespace uavcan_esp32
{

int CanIface::init(const char* devname)
{
	if (fd_ >= 0) {
		::close(fd_);
		fd_ = -1;
	}

	fd_ = ::open(devname, O_RDWR | O_NONBLOCK);

	if (fd_ < 0) {
		PX4_ERR("UAVCAN: Failed to open %s (errno %d)", devname, errno);
		return -errno;
	}

	PX4_INFO("UAVCAN: Successfully opened %s (fd=%d)", devname, fd_);
	error_cnt_ = 0;
	rx_overflow_cnt_ = 0;
	return 0;
}

int16_t CanIface::send(const uavcan::CanFrame& frame, uavcan::MonotonicTime, uavcan::CanIOFlags)
{
	if (fd_ < 0) {
		return -1;
	}

	struct can_msg_s msg;
	std::memset(&msg, 0, sizeof(msg));

	msg.cm_hdr.ch_id    = frame.id & uavcan::CanFrame::MaskExtID;
	msg.cm_hdr.ch_dlc   = frame.dlc;
	msg.cm_hdr.ch_rtr   = (frame.id & uavcan::CanFrame::FlagRTR) ? 1 : 0;
	msg.cm_hdr.ch_extid = (frame.id & uavcan::CanFrame::FlagEFF) ? 1 : 0;

	const uint8_t len = (frame.dlc > 8) ? 8 : frame.dlc;
	std::memcpy(msg.cm_data, frame.data, len);

	ssize_t res = ::write(fd_, &msg, sizeof(msg));

	if (res < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			return 0; // Transmit FIFO full; retry later
		}
		error_cnt_++;
		return -1;
	}

	return 1;
}

int16_t CanIface::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_mono,
                         uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
	if (fd_ < 0) {
		return -1;
	}

	struct can_msg_s msg;
	std::memset(&msg, 0, sizeof(msg));

	ssize_t res = ::read(fd_, &msg, sizeof(msg));

	if (res < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			return 0; // No message waiting
		}
		error_cnt_++;
		return -1;
	}

	if (res == 0 || res < static_cast<ssize_t>(sizeof(msg.cm_hdr))) {
		return 0; // Incomplete or empty read
	}

	out_ts_mono = SystemClock::instance().getMonotonic();
	out_ts_utc  = SystemClock::instance().getUtc();
	out_flags   = 0;

	out_frame.id = msg.cm_hdr.ch_id;
	if (msg.cm_hdr.ch_extid) {
		out_frame.id |= uavcan::CanFrame::FlagEFF;
	}
	if (msg.cm_hdr.ch_rtr) {
		out_frame.id |= uavcan::CanFrame::FlagRTR;
	}

	out_frame.dlc = msg.cm_hdr.ch_dlc;
	const uint8_t len = (msg.cm_hdr.ch_dlc > 8) ? 8 : msg.cm_hdr.ch_dlc;
	std::memcpy(out_frame.data, msg.cm_data, len);

	return 1;
}

int CanDriver::init(uint8_t num_ifaces, uint32_t bitrate)
{
	(void)bitrate;
	PX4_INFO("UAVCAN: CanDriver::init called with %u ifaces", num_ifaces);

	num_ifaces_ = (num_ifaces > 2) ? 2 : num_ifaces;
	if (num_ifaces_ == 0) {
		num_ifaces_ = 1;
	}

	if (num_ifaces_ >= 1) {
		int ret0 = iface0_.init("/dev/can0");
		if (ret0 < 0) {
			PX4_ERR("UAVCAN: Failed to open /dev/can0 (%d)", ret0);
			return -1;
		}
		PX4_INFO("UAVCAN: /dev/can0 opened successfully (fd=%d)", iface0_.getFd());
	}

	if (num_ifaces_ >= 2) {
		int ret1 = iface1_.init("/dev/can1");
		if (ret1 < 0) {
			PX4_ERR("UAVCAN: Failed to open /dev/can1 (%d)", ret1);
			return -1;
		}
		PX4_INFO("UAVCAN: /dev/can1 opened successfully (fd=%d)", iface1_.getFd());
	}

	return 0;
}

int16_t CanDriver::select(uavcan::CanSelectMasks& inout_masks,
                          const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces],
                          uavcan::MonotonicTime blocking_deadline)
{
	struct pollfd fds[2];
	std::memset(fds, 0, sizeof(fds));

	for (uint8_t i = 0; i < num_ifaces_; i++) {
		fds[i].fd = (i == 0) ? iface0_.getFd() : iface1_.getFd();

		if (inout_masks.read & (1U << i)) {
			fds[i].events |= POLLIN;
		}
		if ((inout_masks.write & (1U << i)) && pending_tx[i]) {
			fds[i].events |= POLLOUT;
		}
	}

	const uavcan::MonotonicTime now = SystemClock::instance().getMonotonic();
	int timeout_ms = 0;
	if (blocking_deadline > now) {
		timeout_ms = static_cast<int>((blocking_deadline - now).toMSec());
	}

	int res = ::poll(fds, num_ifaces_, timeout_ms);

	if (res < 0) {
		inout_masks.read = 0;
		inout_masks.write = 0;
		if (errno == EINTR) {
			return 0; // Interrupted by scheduler, safe to continue
		}
		return -1;
	}

	if (res == 0) {
		inout_masks.read = 0;
		inout_masks.write = 0;
		return 0; // Clean timeout
	}

	inout_masks.read = 0;
	inout_masks.write = 0;

	for (uint8_t i = 0; i < num_ifaces_; i++) {
		if (fds[i].revents & (POLLIN | POLLERR | POLLHUP)) {
			inout_masks.read |= (1U << i);
		}
		if (fds[i].revents & POLLOUT) {
			inout_masks.write |= (1U << i);
		}
	}

	return 1;
}

} // namespace uavcan_esp32
