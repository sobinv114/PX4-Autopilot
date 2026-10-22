#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan_esp32/build_config.hpp>
#include <uavcan_esp32/thread.hpp>
#include <nuttx/can/can.h>
#include <fcntl.h>
#include <poll.h>

namespace uavcan_esp32
{

class CanIface : public uavcan::ICanIface, uavcan::Noncopyable
{
	int fd_{-1};
	uint8_t self_index_{0};
	uint64_t error_cnt_{0};
	uint64_t rx_overflow_cnt_{0};

public:
	CanIface(uint8_t index) : self_index_(index) {}
	~CanIface() override { if (fd_ >= 0) { ::close(fd_); } }

	int init(const char* devname);

	int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags) override;
	int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_mono, uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags) override;
	int16_t configureFilters(const uavcan::CanFilterConfig*, uint16_t) override { return 0; }

	/* Pure virtual in ICanIface */
	uint16_t getNumFilters() const override { return 0; }
	uint64_t getErrorCount() const override { return error_cnt_; }
	uint64_t getRxQueueOverflowCount() const { return rx_overflow_cnt_; }

	int getFd() const { return fd_; }
};

class CanDriver : public uavcan::ICanDriver, uavcan::Noncopyable
{
	CanIface iface0_{0};
	CanIface iface1_{1};
	uint8_t num_ifaces_{0};
	BusEvent update_event_{};

public:
	CanDriver() = default;
	~CanDriver() override = default;

	/* Required by UavcanNode::Run() */
	BusEvent& updateEvent() { return update_event_; }

	int init(uint8_t num_ifaces, uint32_t bitrate);

	int16_t select(uavcan::CanSelectMasks& inout_masks,
	               const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces],
	               uavcan::MonotonicTime blocking_deadline) override;

	uavcan::ICanIface* getIface(uint8_t iface_index) override
	{
		if (iface_index == 0) { return &iface0_; }
		if (iface_index == 1) { return &iface1_; }
		return nullptr;
	}

	const uavcan::ICanIface* getIface(uint8_t iface_index) const override
	{
		if (iface_index == 0) { return &iface0_; }
		if (iface_index == 1) { return &iface1_; }
		return nullptr;
	}

	uint8_t getNumIfaces() const override { return num_ifaces_; }

	bool hadActivity() { return true; }
	bool hasHwTimestamps() const { return false; }
};

/**
 * Helper class expected by uavcan_main.cpp
 */
template <unsigned RxQueueCapacity = 128>
class CanInitHelper
{
public:
	CanDriver driver;

	CanInitHelper(uint16_t /* ifaces_mask */ = 0) {}

	int init(uavcan::uint32_t bitrate)
	{
		return driver.init(UAVCAN_ESP32_NUM_IFACES, bitrate);
	}
};

} // namespace uavcan_esp32
