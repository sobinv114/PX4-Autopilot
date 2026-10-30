/****************************************************************************
 *
 *   Bosch BMP180 barometer driver (I2C only).
 *   Structure follows the in-tree bmp388 driver.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <stddef.h>
#include <stdint.h>

#include "bmp180_math.h"

namespace Bosch_BMP180
{

static constexpr uint32_t I2C_SPEED = 100 * 1000;
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x77; // 0xEE / 0xEF in the datasheet is the 8 bit form
static constexpr uint8_t BMP180_CHIP_ID = 0x55;

// Pressure oversampling_setting: 0 ultra low power, 1 standard, 2 high, 3 ultra high resolution.
static constexpr uint8_t OSS = 3;
static_assert(OSS <= 3, "OSS must be 0..3");

// DS table 1/3: pressure conversion time per OSS (typical / maximum), temperature 3 / 4.5 ms
static constexpr uint32_t PRESSURE_CONVERSION_TYP_US[4] {3000, 5000, 9000, 17000};
static constexpr uint32_t PRESSURE_CONVERSION_MAX_US[4] {4500, 7500, 13500, 25500};
static constexpr uint32_t TEMP_CONVERSION_TYP_US = 3000;
static constexpr uint32_t TEMP_CONVERSION_MAX_US = 4500;

// DS 3.3: it is sufficient to measure the temperature once per second
static constexpr uint64_t TEMPERATURE_INTERVAL_US = 1000000;

enum class Register : uint8_t {
	CALIB_START = 0xAA, // AC1 MSB .. MD LSB = 0xBF
	CHIP_ID = 0xD0,
	SOFT_RESET = 0xE0,
	CTRL_MEAS = 0xF4,
	OUT_MSB = 0xF6, // 0xF6 MSB, 0xF7 LSB, 0xF8 XLSB
};

static constexpr uint8_t SOFT_RESET_VALUE = 0xB6;
static constexpr uint8_t CTRL_MEAS_SCO = 0x20; // stays 1 while a conversion is running
static constexpr uint8_t CMD_TEMPERATURE = 0x2E;
static constexpr uint8_t CMD_PRESSURE = 0x34; // | (OSS << 6)

} // namespace Bosch_BMP180

using namespace Bosch_BMP180;

class BMP180 : public device::I2C, public I2CSPIDriver<BMP180>
{
public:
	BMP180(const I2CSPIDriverConfig &config);
	~BMP180() override;

	static void print_usage();

	int init() override;
	void print_status() override;

	void RunImpl();

private:
	int probe() override;

	bool ReadCalibration();
	bool StartConversion(uint8_t command, uint32_t typ_us, uint32_t max_us);
	int CheckConversion();
	void Failure();
	uint32_t RetryDelay();

	bool RegisterRead(Register reg, void *buf, size_t len);
	uint8_t RegisterRead(Register reg);
	bool RegisterWrite(Register reg, uint8_t value);

	PX4Barometer _px4_baro;

	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _conversion_timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": conversion timeout")};
	perf_counter_t _bad_data_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad data")};
	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		MEASURE_TEMPERATURE,
		COLLECT_TEMPERATURE,
		MEASURE_PRESSURE,
		COLLECT_PRESSURE,
	} _state{STATE::RESET};

	hrt_abstime _measure_timestamp{0};
	hrt_abstime _temperature_timestamp{0};
	uint32_t _conversion_typ_us{0};
	uint32_t _conversion_max_us{0};

	int _failure_count{0};
	int _configure_failures{0};
	uint8_t _reset_streak{0};

	int32_t _b5{0};
	int32_t _temperature_0p1c{0};

	calib_data _calib{};
};
