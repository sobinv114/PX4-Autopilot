/****************************************************************************
 *
 *   Bosch BMP180 barometer driver (I2C only).
 *
 ****************************************************************************/

#include "bmp180.h"

#include <drivers/drv_sensor.h>
#include <px4_platform_common/module.h>

#include <string.h>

void BMP180::print_usage()
{
	PRINT_MODULE_USAGE_NAME("bmp180", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("baro");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x77);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int bmp180_main(int argc, char *argv[])
{
	using ThisDriver = BMP180;
	BusCLIArguments cli{true, false};
	cli.i2c_address = I2C_ADDRESS_DEFAULT;
	cli.default_i2c_frequency = I2C_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BARO_DEVTYPE_BMP180);

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
