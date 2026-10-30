/****************************************************************************
 *
 *   Bosch BMP180 barometer driver (I2C only).
 *
 ****************************************************************************/

#include "bmp180.h"

#include <errno.h>
#include <inttypes.h>

using namespace time_literals;

BMP180::BMP180(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_baro(get_device_id(), config.external)
{
	_retries = 1;
}

BMP180::~BMP180()
{
	perf_free(_bad_transfer_perf);
	perf_free(_conversion_timeout_perf);
	perf_free(_bad_data_perf);
	perf_free(_reset_perf);
}

int BMP180::init()
{
	const int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	_state = STATE::RESET;
	ScheduleNow();

	return PX4_OK;
}

int BMP180::probe()
{
	for (int i = 0; i < 3; i++) {
		if (RegisterRead(Register::CHIP_ID) == BMP180_CHIP_ID) {
			return PX4_OK;
		}

		DEVICE_DEBUG("unexpected CHIP_ID");
	}

	return PX4_ERROR;
}

void BMP180::RunImpl()
{
	switch (_state) {
	case STATE::RESET:
		RegisterWrite(Register::SOFT_RESET, SOFT_RESET_VALUE);
		perf_count(_reset_perf);
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(10_ms); // DS table 6: start-up time 10 ms
		break;

	case STATE::WAIT_FOR_RESET:
		if (RegisterRead(Register::CHIP_ID) == BMP180_CHIP_ID) {
			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else {
			_state = STATE::RESET;
			ScheduleDelayed(RetryDelay());
		}

		break;

	case STATE::CONFIGURE:
		if (ReadCalibration()) {
			_configure_failures = 0;
			_reset_streak = 0;
			_failure_count = 0;
			_state = STATE::MEASURE_TEMPERATURE;
			ScheduleNow();

		} else {
			// init() returns before the first configure, so this is the only place a dead sensor is reported
			if (++_configure_failures == 10) {
				PX4_ERR("calibration read failed, not publishing");
			}

			_state = STATE::RESET;
			ScheduleDelayed(RetryDelay());
		}

		break;

	case STATE::MEASURE_TEMPERATURE:
		if (StartConversion(CMD_TEMPERATURE, TEMP_CONVERSION_TYP_US, TEMP_CONVERSION_MAX_US)) {
			_state = STATE::COLLECT_TEMPERATURE;
			ScheduleDelayed(_conversion_typ_us);

		} else {
			Failure();
		}

		break;

	case STATE::COLLECT_TEMPERATURE:
		switch (CheckConversion()) {
		case PX4_OK: {
				uint8_t buf[2] {};

				if (RegisterRead(Register::OUT_MSB, buf, sizeof(buf))) {
					const int32_t ut = ((int32_t)buf[0] << 8) | buf[1];

					if (compensate_temperature(_calib, ut, _b5, _temperature_0p1c)) {
						_temperature_timestamp = hrt_absolute_time();
						_state = STATE::MEASURE_PRESSURE;
						ScheduleNow();
						break;
					}

					perf_count(_bad_data_perf);
				}

				Failure();
			}
			break;

		case -EAGAIN:
			ScheduleDelayed(1_ms);
			break;

		default:
			Failure();
			break;
		}

		break;

	case STATE::MEASURE_PRESSURE:
		if (StartConversion(CMD_PRESSURE | (OSS << 6), PRESSURE_CONVERSION_TYP_US[OSS], PRESSURE_CONVERSION_MAX_US[OSS])) {
			_state = STATE::COLLECT_PRESSURE;
			ScheduleDelayed(_conversion_typ_us);

		} else {
			Failure();
		}

		break;

	case STATE::COLLECT_PRESSURE:
		switch (CheckConversion()) {
		case PX4_OK: {
				uint8_t buf[3] {};

				if (RegisterRead(Register::OUT_MSB, buf, sizeof(buf))) {
					// UP = (MSB << 16 | LSB << 8 | XLSB) >> (8 - oss)
					const int32_t up = (((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2]) >> (8 - OSS);
					int32_t pressure_pa = 0;

					// DS: 300..1100 hPa, with margin
					if (compensate_pressure(_calib, _b5, up, OSS, pressure_pa)
					    && (pressure_pa > 20000) && (pressure_pa < 120000)) {

						// the sample is integrated over the conversion window, so timestamp its midpoint
						const hrt_abstime timestamp_sample = _measure_timestamp + (_conversion_typ_us / 2);

						_px4_baro.set_error_count(perf_event_count(_bad_transfer_perf) + perf_event_count(_conversion_timeout_perf)
									  + perf_event_count(_bad_data_perf));
						_px4_baro.set_temperature(_temperature_0p1c / 10.f);
						_px4_baro.update(timestamp_sample, (float)pressure_pa); // Pa

						if (_failure_count > 0) {
							_failure_count--;
						}

						_state = (hrt_elapsed_time(&_temperature_timestamp) >= TEMPERATURE_INTERVAL_US)
							 ? STATE::MEASURE_TEMPERATURE : STATE::MEASURE_PRESSURE;
						ScheduleNow();
						break;
					}

					perf_count(_bad_data_perf);
				}

				Failure();
			}
			break;

		case -EAGAIN:
			ScheduleDelayed(1_ms);
			break;

		default:
			Failure();
			break;
		}

		break;
	}
}

bool BMP180::StartConversion(uint8_t command, uint32_t typ_us, uint32_t max_us)
{
	if (!RegisterWrite(Register::CTRL_MEAS, command)) {
		return false;
	}

	// the conversion starts when the write lands, not when this work item started
	_measure_timestamp = hrt_absolute_time();
	_conversion_typ_us = typ_us;
	_conversion_max_us = max_us;
	return true;
}

// PX4_OK when the conversion is done, -EAGAIN while it is still running
int BMP180::CheckConversion()
{
	uint8_t ctrl = 0;

	if (!RegisterRead(Register::CTRL_MEAS, &ctrl, sizeof(ctrl))) {
		return -EIO;
	}

	if (ctrl & CTRL_MEAS_SCO) {
		if (hrt_elapsed_time(&_measure_timestamp) < _conversion_max_us) {
			return -EAGAIN;
		}

		perf_count(_conversion_timeout_perf);
		return -ETIMEDOUT;
	}

	return PX4_OK;
}

void BMP180::Failure()
{
	// skip the sample; reset once failures are persistent
	if (++_failure_count > 10) {
		_state = STATE::RESET;
		ScheduleDelayed(RetryDelay());

	} else {
		// restart from a fresh temperature so b5 is never stale after an error
		_state = STATE::MEASURE_TEMPERATURE;
		ScheduleDelayed(10_ms);
	}
}

// Retry every 100 ms at first, then once per second if the sensor stays dead.
// This keeps a missing or wedged sensor from hogging a shared I2C bus.
uint32_t BMP180::RetryDelay()
{
	if (_reset_streak < 255) {
		_reset_streak++;
	}

	return (_reset_streak > 20) ? 1000000 : 100000;
}

bool BMP180::ReadCalibration()
{
	uint8_t buf[CALIB_SIZE] {};

	for (uint8_t i = 0; i < 11; i++) {
		const uint8_t reg = static_cast<uint8_t>(Register::CALIB_START) + (i * 2);

		if (transfer(&reg, 1, &buf[i * 2], 2) != PX4_OK) {
			perf_count(_bad_transfer_perf);
			return false;
		}
	}

	if (!parse_calibration(buf, _calib)) {
		PX4_ERR("calibration word is 0 or 0xFFFF");
		return false;
	}

	return true;
}

bool BMP180::RegisterRead(Register reg, void *buf, size_t len)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);

	if (transfer(&cmd, 1, static_cast<uint8_t *>(buf), len) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	return true;
}

uint8_t BMP180::RegisterRead(Register reg)
{
	uint8_t value = 0;
	RegisterRead(reg, &value, 1);
	return value;
}

bool BMP180::RegisterWrite(Register reg, uint8_t value)
{
	const uint8_t buf[2] {static_cast<uint8_t>(reg), value};

	if (transfer(buf, sizeof(buf), nullptr, 0) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	return true;
}

void BMP180::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("oss: %d, last temperature: %.1f degC", (int)OSS, (double)(_temperature_0p1c / 10.f));

	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_conversion_timeout_perf);
	perf_print_counter(_bad_data_perf);
	perf_print_counter(_reset_perf);
}
