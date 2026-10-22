#pragma once

#include <uavcan/uavcan.hpp>
#include <drivers/drv_hrt.h>
#include <time.h>

namespace uavcan_esp32
{

namespace clock
{

inline uavcan::MonotonicTime getMonotonic()
{
	return uavcan::MonotonicTime::fromUSec(hrt_absolute_time());
}

inline uavcan::UtcTime getUtc()
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return uavcan::UtcTime::fromUSec(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
}

inline void adjustUtc(uavcan::UtcDuration)
{
	// Optional UTC adjustment
}

class SystemClock : public uavcan::ISystemClock, uavcan::Noncopyable
{
public:
	uavcan::MonotonicTime getMonotonic() const override
	{
		return clock::getMonotonic();
	}

	uavcan::UtcTime getUtc() const override
	{
		return clock::getUtc();
	}

	void adjustUtc(uavcan::UtcDuration adjustment) override
	{
		clock::adjustUtc(adjustment);
	}

	static SystemClock& instance()
	{
		static SystemClock inst;
		return inst;
	}
};

} // namespace clock

using clock::SystemClock;

} // namespace uavcan_esp32
