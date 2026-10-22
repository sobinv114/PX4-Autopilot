/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once
#include <inttypes.h>
#include <stdint.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/sensor_temp.h>
#include <uORB/topics/sensor_hygrometer.h>
#include <uORB/PublicationMulti.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

#define AHT10_I2C_ADDR_DEFAULT      0x38

#define AHT10_CMD_INIT              0xE1
#define AHT10_CMD_TRIGGER_MEASURE   0xAC
#define AHT10_CMD_SOFT_RESET        0xBA

#define AHT10_STATUS_BUSY           0x80
#define AHT10_STATUS_CALIBRATED     0x08

class AHT10 : public device::I2C, public I2CSPIDriver<AHT10>
{
public:
	AHT10(const I2CSPIDriverConfig &config);
	~AHT10() override;

	int init() override;
	int probe() override;
	void RunImpl();
	static void print_usage();

protected:
	void print_status();

private:
	enum class State {
		Trigger,
		Read
	};

	int trigger_measurement();
	int read_and_publish();

	uORB::PublicationMulti<sensor_temp_s> _sensor_temp_pub{ORB_ID(sensor_temp)};
	uORB::PublicationMulti<sensor_hygrometer_s> _sensor_hygro_pub{ORB_ID(sensor_hygrometer)};
	sensor_temp_s _sensor_temp{};
	sensor_hygrometer_s _sensor_hygro{};


	State _state{State::Trigger};

	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;
};
