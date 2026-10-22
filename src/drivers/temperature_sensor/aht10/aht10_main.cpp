/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file aht10_main.cpp
 * Driver for the Aosong AHT10 Temperature and Humidity Sensor via I2C.
 */

#include "aht10.h"
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>

void AHT10::print_usage()
{
	PRINT_MODULE_USAGE_NAME("aht10", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(AHT10_I2C_ADDR_DEFAULT);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void AHT10::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_errors);

	// Add this to print the live temperature whenever you type "aht10 status"
	if (_sensor_temp.timestamp != 0) {
		PX4_INFO("Latest Temperature: %.2f C", (double)_sensor_temp.temperature);
	} else {
		PX4_INFO("No temperature data yet.");
	}

	if (_sensor_hygro.timestamp != 0) {
		PX4_INFO("Latest: %.2f C, %.1f %%RH", (double)_sensor_hygro.temperature, (double)_sensor_hygro.humidity);
	} else {
		PX4_INFO("No humidity data yet.");
	}

	// uORB::Subscription sub{ORB_ID(sensor_temp)};
	// sensor_temp_s msg{};

	// // if (sub.copy(&msg)) {
	// // 	PX4_INFO("sub.copy OK: %.2f C", (double)msg.temperature);

	// // } else {
	// // 	PX4_INFO("sub.copy failed");
	// // }
}

extern "C" int aht10_main(int argc, char *argv[])
{
	using ThisDriver = AHT10;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = AHT10_I2C_ADDR_DEFAULT;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_TEMP_DEVTYPE_AHT10);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
