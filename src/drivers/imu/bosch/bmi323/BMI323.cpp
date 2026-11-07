/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * (BSD-3-Clause, see Bosch_BMI323_registers.hpp for full text)
 *
 ****************************************************************************/

#include "BMI323.hpp"

#include <string.h>

#define BMI323_DEBUG // enables CheckErrorRegister() decoding (costs one extra SPI read per FIFO read)

using namespace time_literals;

static constexpr size_t CFG_CHUNK_BYTES = 32; // bytes per FEATURE_DATA_TX burst
static constexpr size_t CFG_DATA_START = 4;    // first 4 bytes of each array are header

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

/**
* The following configuration array has the following copyright:
*
* Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Source: Bosch BMI323 SensorAPI (github.com/boschsensortec/BMI323_SensorAPI), bmi3.c
*/
// Layout of code/table: [base_addr_lo, base_addr_hi, len_lo, len_hi, data...]; data is uploaded in 16 bit words
/***************************************************************************/

/*!              Static Variable
 ****************************************************************************/

/*! Array to store config array code */
static const uint8_t bmi323_config_array_code[] = {
    0x48, 0x02, 0x94, 0x01, 0x01, 0x2e, 0xa0, 0xf2, 0x09, 0xbc, 0x20, 0x50, 0x0d, 0xb8, 0x03, 0x2e, 0xd0, 0x03, 0x01,
    0x1a, 0xf0, 0x7f, 0xeb, 0x7f, 0x17, 0x2f, 0x20, 0x30, 0x21, 0x2e, 0xd2, 0x01, 0x98, 0x2e, 0x1d, 0xc7, 0x98, 0x2e,
    0x2f, 0xc7, 0x98, 0x2e, 0x8e, 0xc0, 0x03, 0x2e, 0xf6, 0x01, 0x12, 0x24, 0xff, 0xe3, 0x4a, 0x08, 0x00, 0x30, 0x21,
    0x2e, 0xcd, 0x01, 0xf2, 0x6f, 0x21, 0x2e, 0xd1, 0x01, 0x25, 0x2e, 0xd0, 0x03, 0x23, 0x2e, 0xf6, 0x01, 0xeb, 0x6f,
    0xe0, 0x5f, 0xb8, 0x2e, 0x01, 0x2e, 0xac, 0x01, 0x01, 0xb2, 0x06, 0x2f, 0x01, 0x2e, 0x21, 0xf1, 0xb1, 0x3f, 0x01,
    0x08, 0xc0, 0x2e, 0x21, 0x2e, 0x21, 0xf1, 0x01, 0x2e, 0x21, 0xf1, 0x41, 0x30, 0x01, 0x0a, 0xc0, 0x2e, 0x21, 0x2e,
    0x21, 0xf1, 0xb8, 0x2e, 0x01, 0x2e, 0xf6, 0x01, 0x86, 0xbc, 0x9f, 0xb8, 0x41, 0x90, 0x06, 0x2f, 0x11, 0x24, 0xff,
    0xfd, 0x01, 0x08, 0x21, 0x2e, 0xf6, 0x01, 0x80, 0x2e, 0x43, 0xeb, 0xb8, 0x2e, 0x10, 0x50, 0xfb, 0x7f, 0x98, 0x2e,
    0x48, 0x02, 0xfb, 0x6f, 0xf0, 0x5f, 0x80, 0x2e, 0xeb, 0x02, 0x80, 0x2e, 0x48, 0x02, 0x01, 0x2e, 0xd1, 0x03, 0x10,
    0x50, 0x00, 0x90, 0x0e, 0x2f, 0x01, 0x2e, 0xa0, 0xf2, 0x09, 0xbc, 0x0d, 0xb8, 0x31, 0x30, 0x08, 0x04, 0xfb, 0x7f,
    0x21, 0x2e, 0xd5, 0x01, 0x98, 0x2e, 0x18, 0xe5, 0x10, 0x30, 0x21, 0x2e, 0xd1, 0x03, 0xfb, 0x6f, 0xf0, 0x5f, 0xb8,
    0x2e, 0x03, 0x2e, 0x05, 0xf8, 0x9a, 0xbc, 0x10, 0x50, 0x9f, 0xb8, 0x40, 0x90, 0xfb, 0x7f, 0x05, 0x2f, 0x03, 0x2e,
    0x05, 0xf8, 0x02, 0x32, 0x4a, 0x0a, 0x23, 0x2e, 0x05, 0xf8, 0x05, 0x2e, 0xf6, 0x01, 0xab, 0xbc, 0x9f, 0xb8, 0x41,
    0x90, 0x22, 0x2f, 0x03, 0x2e, 0xae, 0x00, 0x9a, 0xbc, 0x9f, 0xb8, 0x41, 0x90, 0x1c, 0x2f, 0x14, 0x24, 0x16, 0xf1,
    0xf0, 0x3e, 0x02, 0x08, 0x11, 0x41, 0x02, 0x41, 0x21, 0x2e, 0xf6, 0x01, 0x09, 0x2e, 0xad, 0x00, 0x13, 0x24, 0xac,
    0x00, 0x98, 0x2e, 0xad, 0x03, 0x12, 0x30, 0x11, 0x24, 0xf9, 0x01, 0xc2, 0x08, 0x44, 0x40, 0x10, 0x24, 0xff, 0xdf,
    0xbd, 0xbd, 0x04, 0x09, 0x1c, 0x0b, 0x54, 0x42, 0x00, 0x2e, 0x44, 0x40, 0x94, 0x0a, 0x42, 0x42, 0x00, 0x2e, 0xfb,
    0x6f, 0xf0, 0x5f, 0xb8, 0x2e, 0xb8, 0x2e, 0x01, 0x2e, 0x12, 0xf1, 0x03, 0x2e, 0xbc, 0x00, 0x96, 0xbc, 0x96, 0xb8,
    0x54, 0xb0, 0x20, 0x50, 0x2d, 0x2e, 0xd2, 0x03, 0x00, 0xa6, 0x21, 0x2e, 0xd3, 0x03, 0xfb, 0x7f, 0x01, 0x30, 0x07,
    0x30, 0x58, 0x2f, 0x09, 0x2e, 0xd5, 0x03, 0x01, 0xb3, 0x13, 0x30, 0x05, 0x2e, 0xd2, 0x03, 0x06, 0x2f, 0x0b, 0x2e,
    0xd7, 0x03, 0x45, 0x0e, 0x02, 0x2f, 0x42, 0x01, 0x2b, 0x2e, 0xd7, 0x03, 0x0b, 0x2e, 0xd7, 0x03, 0xc5, 0x0e, 0x11,
    0x2f, 0x00, 0x91, 0x0f, 0x2f, 0x27, 0x2e, 0xd5, 0x03, 0x62, 0x25, 0x44, 0x31, 0x05, 0x30, 0x41, 0x8b, 0x20, 0x19,
    0x50, 0xa1, 0xfb, 0x2f, 0xf4, 0x3f, 0x20, 0x05, 0x94, 0x04, 0x25, 0x2e, 0xd8, 0x03, 0x2d, 0x2e, 0xd6, 0x03, 0x05,
    0x2e, 0xd5, 0x03, 0x81, 0x90, 0xe0, 0x7f, 0x07, 0x2f, 0x05, 0x2e, 0xd4, 0x03, 0xd0, 0x0e, 0x03, 0x2f, 0x23, 0x2e,
    0xd5, 0x03, 0x27, 0x2e, 0xd9, 0x03, 0x05, 0x2e, 0xd5, 0x03, 0x81, 0x90, 0x06, 0x2f, 0x05, 0x2e, 0xbc, 0x00, 0x13,
    0x24, 0x00, 0xfc, 0x93, 0x08, 0x25, 0x2e, 0xbc, 0x00, 0x05, 0x2e, 0xd9, 0x03, 0x81, 0x90, 0x15, 0x2f, 0x05, 0x2e,
    0xd8, 0x03, 0x42, 0x0e, 0x11, 0x2f, 0x23, 0x2e, 0xd9, 0x03, 0x50, 0x30, 0x98, 0x2e, 0x04, 0xeb, 0x01, 0x2e, 0xd6,
    0x03, 0x05, 0x2e, 0xbc, 0x00, 0x11, 0x24, 0xff, 0x03, 0x13, 0x24, 0x00, 0xfc, 0x93, 0x08, 0x01, 0x08, 0x02, 0x0a,
    0x21, 0x2e, 0xbc, 0x00, 0xe0, 0x6f, 0x21, 0x2e, 0xd4, 0x03, 0xfb, 0x6f, 0xe0, 0x5f, 0xb8, 0x2e, 0x50, 0x50, 0x0a,
    0x25, 0x3b, 0x80, 0xe3, 0x7f, 0x02, 0x42, 0xc1, 0x7f, 0xf1, 0x7f, 0xdb, 0x7f, 0x22, 0x30, 0x03, 0x30, 0x10, 0x25,
    0x98, 0x2e, 0x8e, 0xdf, 0xf1, 0x6f, 0x41, 0x16, 0x81, 0x08, 0xe3, 0x6f, 0x4b, 0x08, 0x0a, 0x1a, 0xdb, 0x6f, 0x11,
    0x30, 0x03, 0x30, 0xc0, 0x2e, 0x19, 0x22, 0xb0, 0x5f, 0x40, 0x50, 0x0a, 0x25, 0x3c, 0x80, 0xfb, 0x7f, 0x01, 0x42,
    0xd2, 0x7f, 0xe3, 0x7f, 0x32, 0x30, 0x03, 0x30, 0x10, 0x25, 0x98, 0x2e, 0x8e, 0xdf, 0xfb, 0x6f, 0xc0, 0x5f, 0xb8,
    0x2e, 0x02, 0x25, 0x23, 0x25, 0x30, 0x25, 0x50, 0x50, 0x00, 0x30, 0xbb, 0x7f, 0x00, 0x43, 0xf4, 0x7f, 0xe2, 0x7f,
    0xd3, 0x7f, 0xc1, 0x7f, 0x98, 0x2e, 0x58, 0x03, 0x01, 0xb2, 0x19, 0x2f, 0xc1, 0x6f, 0xf2, 0x30, 0x8a, 0x08, 0x13,
    0x24, 0xf0, 0x00, 0x4b, 0x08, 0x24, 0xbd, 0x94, 0xb8, 0x4a, 0x0a, 0x81, 0x16, 0xd3, 0x6f, 0xe4, 0x6f, 0x93, 0x08,
    0x4c, 0x08, 0x4a, 0x0a, 0x12, 0x24, 0x00, 0xff, 0x8a, 0x08, 0x13, 0x24, 0xff, 0x00, 0x4b, 0x08, 0x98, 0xbc, 0x28,
    0xb9, 0xf3, 0x6f, 0x4a, 0x0a, 0xc1, 0x42, 0x00, 0x2e, 0xbb, 0x6f, 0xb0, 0x5f, 0xb8, 0x2e, 0x20, 0x50, 0x01, 0x2e,
    0xda, 0x03, 0xf3, 0x7f, 0x00, 0xb2, 0xeb, 0x7f, 0xa3, 0x32, 0x0e, 0x2f, 0x01, 0x90, 0x0a, 0x2f, 0x24, 0x25, 0x07,
    0x2e, 0xdb, 0x03, 0xf4, 0x6f, 0xa1, 0x32, 0x98, 0x2e, 0x80, 0x03, 0x00, 0x30, 0x21, 0x2e, 0xda, 0x03, 0x0b, 0x2d,
    0x0a, 0x2c, 0x00, 0x30, 0x98, 0x2e, 0x71, 0x03, 0xf4, 0x6f, 0x21, 0x2e, 0xdb, 0x03, 0x00, 0x43, 0x10, 0x30, 0x21,
    0x2e, 0xda, 0x03, 0xeb, 0x6f, 0xe0, 0x5f, 0xb8, 0x2e, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*! Array to store config array table */
static const uint8_t bmi323_config_array_table[] = {
    0x05, 0x02, 0x0c, 0x00, 0x99, 0x02, 0x97, 0x02, 0x8f, 0x02, 0x6e, 0x02, 0x00, 0x00, 0x80, 0x02, 0xaf, 0x02, 0xea,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x02
};

/*! Array to store config version */
static const uint8_t bmi323_config_version[] = {
    0xad, 0x00, 0x01, 0x00, 0x08, 0x08
};

BMI323::BMI323(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation, config.external),
	_px4_gyro(get_device_id(), config.rotation, config.external)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

BMI323::~BMI323()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_dummy_frame_perf);
	perf_free(_drdy_missed_perf);
}

int BMI323::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	PX4_DEBUG("init function called, resetting...");

	return Reset() ? 0 : -1;
}

bool BMI323::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

// Debug decoder for ERR_REG (datasheet register 0x01)
void BMI323::CheckErrorRegister()
{
#ifdef BMI323_DEBUG
	const uint16_t err = RegisterRead(Register::ERR_REG);

	if (err & ERR_REG_BIT::fatal_err) {
		PX4_DEBUG("BMI323: fatal_err (chip not operational, soft reset needed)");
	}

	if (err & ERR_REG_BIT::feat_eng_ovrld) {
		PX4_DEBUG("BMI323: feature engine overload");
	}

	if (err & ERR_REG_BIT::feat_eng_wd) {
		PX4_DEBUG("BMI323: feature engine watchdog");
	}

	if (err & ERR_REG_BIT::acc_conf_err) {
		PX4_DEBUG("BMI323: unsupported accel configuration (ACC_CONF)");
	}

	if (err & ERR_REG_BIT::gyr_conf_err) {
		PX4_DEBUG("BMI323: unsupported gyro configuration (GYR_CONF)");
	}

	if (err & (ERR_REG_BIT::i3c_error0 | ERR_REG_BIT::i3c_error3)) {
		PX4_DEBUG("BMI323: I3C error 0x%04x", err);
	}

#endif
}

void BMI323::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void BMI323::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz), %d samples per transfer",
		 _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us, (int)_fifo_samples);

	PX4_INFO("feature engine: %s, config array: %s",
		 _feature_engine_active ? "active" : "off", _config_array_loaded ? "loaded" : "not loaded");

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_dummy_frame_perf);
	perf_print_counter(_drdy_missed_perf);
}

int BMI323::probe()
{
	// After power-up / soft reset the BMI323 interface is in I2C mode. A rising edge on CSB
	// switches it to SPI after 200 us (datasheet 7.2.1). The first (dummy) read provides that edge.
	RegisterRead(Register::CHIP_ID);
	px4_usleep(250);

	const uint8_t id = RegisterRead(Register::CHIP_ID) & 0xFF;

	PX4_DEBUG("CHIP_ID 0x%02x", id);

	if (id != chip_id) {
		DEVICE_DEBUG("unexpected CHIP_ID 0x%02x", id);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void BMI323::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// soft reset; the device falls back to I2C mode and reverts all registers
		PX4_DEBUG("soft reset");
		RegisterWrite(Register::CMD, CMD_SOFTRESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(5_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// dummy read to re-enter SPI mode (see probe())
		RegisterRead(Register::CHIP_ID);
		px4_usleep(250);

		if ((RegisterRead(Register::CHIP_ID) & 0xFF) == chip_id) {
			PX4_DEBUG("CHIP_ID matches after reset");
			// clear POR flag (clear on read)
			RegisterRead(Register::STATUS);

			_feature_engine_active = false;
			_config_array_loaded = false;

#if BMI323_LOAD_CONFIG_FILE

			// sensors are still disabled after soft reset, which the feature engine needs (datasheet 5.8.1)
			if (EnableFeatureEngineStart()) {
				_feature_engine_timestamp = now;
				_state = STATE::WAIT_FOR_FEATURE_ENGINE;
				ScheduleDelayed(10_ms);
				break;
			}

#endif
			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else {
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::WAIT_FOR_FEATURE_ENGINE:
		if ((RegisterRead(Register::FEATURE_IO1) & FEATURE_IO1_ERROR_STATUS_MASK) == FEATURE_IO1_ENGINE_ACTIVE) {
			PX4_DEBUG("feature engine active");
			_feature_engine_active = true;

			// failure is not fatal: accel/gyro/FIFO work without the configuration array
			_config_array_loaded = LoadConfigArray();

			if (!_config_array_loaded) {
				PX4_DEBUG("configuration array upload failed, continuing without");
			}

			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else if (hrt_elapsed_time(&_feature_engine_timestamp) > 1000_ms) {
			PX4_DEBUG("feature engine did not start, continuing without");
			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else {
			ScheduleDelayed(10_ms);
		}

		break;

	case STATE::CONFIGURE:
		PX4_DEBUG("IMU now in configure state");

		if (Configure()) {
			// gyroscope start-up time is up to 30 ms, give it some margin
			_state = STATE::WAIT_FOR_STARTUP;
			ScheduleDelayed(50_ms);

		} else {
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::WAIT_FOR_STARTUP:
		FIFOReset();

		if (DataReadyInterruptConfigure()) {
			_data_ready_interrupt_enabled = true;

			// backup schedule as a watchdog timeout
			ScheduleDelayed(100_ms);

		} else {
			_data_ready_interrupt_enabled = false;
			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
		}

		_state = STATE::FIFO_READ;
		break;

	case STATE::FIFO_READ: {
			PX4_DEBUG("reading from FIFO");

			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			bool success = false;
			const uint16_t fifo_words = FIFOReadCount();

			if (fifo_words >= (FIFO::SIZE_WORDS - 2 * FIFO::FRAME_WORDS)) {
				// FIFO (almost) full
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else if (fifo_words < FIFO::FRAME_WORDS) {
				perf_count(_fifo_empty_perf);

			} else {
				int32_t samples = fifo_words / FIFO::FRAME_WORDS;

				// tolerate minor jitter, leave sample to next iteration if behind by only 1
				if (samples == _fifo_samples + 1) {
					timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
					samples--;
				}

				if (samples > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (samples >= _fifo_samples) {
					if (FIFORead(timestamp_sample, samples)) {
						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					PX4_DEBUG("failure count > 10, resetting...");
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register]) && CheckErrors()) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					PX4_DEBUG("register check failed, resetting...");

					// register check failed or fatal error flagged, force reset
					perf_count(_bad_register_perf);
					Reset();
					return;
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void BMI323::SetAccelScaleAndRange()
{
	const uint16_t range = RegisterRead(Register::ACC_CONF) & ACC_RANGE::MASK;

	float g = 16.f;

	switch (range) {
	case ACC_RANGE::G2:  g = 2.f;  break;

	case ACC_RANGE::G4:  g = 4.f;  break;

	case ACC_RANGE::G8:  g = 8.f;  break;

	case ACC_RANGE::G16: g = 16.f; break;

	default: break;
	}

	_px4_accel.set_scale(g * CONSTANTS_ONE_G / 32768.f);
	_px4_accel.set_range(g * CONSTANTS_ONE_G);
}

void BMI323::SetGyroScale()
{
	// 16 bit data, +-2000 dps (16.384 LSB/dps)
	_px4_gyro.set_scale(math::radians(2000.0f) / 32768.0f);
	_px4_gyro.set_range(math::radians(2000.0f));
}

void BMI323::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_samples = math::min((float)_fifo_empty_interval_us / (1e6f / RATE), (float)FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual sample limit
	_fifo_empty_interval_us = _fifo_samples * (1e6f / RATE);

	ConfigureFIFOWatermark(_fifo_samples);
}

void BMI323::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO_WATERMARK is 10 bit, unit = 16 bit words, one accel+gyro frame = 6 words
	const uint16_t wm = (samples * FIFO::FRAME_WORDS) & 0x03FF;

	for (auto &r : _register_cfg) {
		if (r.reg == Register::FIFO_WATERMARK) {
			r.set_bits = wm;
			r.clear_bits = (~wm) & 0x03FF;
		}
	}
}

// Burst write: [reg][data...]
void BMI323::RegisterWriteBurst(Register reg, const uint8_t *data, size_t len)
{
	uint8_t buf[1 + CFG_CHUNK_BYTES];

	if (len > CFG_CHUNK_BYTES) {
		return;
	}

	buf[0] = static_cast<uint8_t>(reg);
	memcpy(&buf[1], data, len);
	transfer(buf, nullptr, len + 1);
}

// Datasheet 5.8.1 / Figure 9: FEATURE_IO2 = 0x012C, FEATURE_IO_STATUS = 1, FEATURE_CTRL.engine_en = 1.
// Completion (FEATURE_IO1.error_status == 1) is polled in STATE::WAIT_FOR_FEATURE_ENGINE.
bool BMI323::EnableFeatureEngineStart()
{
	RegisterWrite(Register::FEATURE_IO2, FEATURE_IO2_ENGINE_KEY);
	RegisterWrite(Register::FEATURE_IO_STATUS, FEATURE_IO_STATUS_TRIGGER);
	RegisterWrite(Register::FEATURE_CTRL, FEATURE_CTRL_ENGINE_EN);
	return true;
}

// uploads one array: first 2 bytes = base address (in words), data starts at byte 4
bool BMI323::UploadConfigBlock(const uint8_t *array, size_t size)
{
	uint16_t base = (uint16_t)array[0] | ((uint16_t)array[1] << 8);
	const size_t remain = (size - CFG_DATA_START) % CFG_CHUNK_BYTES;
	const size_t full_end = size - remain;
	size_t i = CFG_DATA_START;

	// full chunks
	for (; i < full_end; i += CFG_CHUNK_BYTES) {
		const uint8_t addr[2] = { (uint8_t)(base & 0xFF), (uint8_t)(base >> 8) };
		RegisterWriteBurst(Register::FEATURE_DATA_ADDR, addr, 2);
		RegisterWriteBurst(Register::FEATURE_DATA_TX, &array[i], CFG_CHUNK_BYTES);
		base += CFG_CHUNK_BYTES / 2;
	}

	// remaining bytes one word at a time
	for (; i < size; i += 2) {
		const uint8_t addr[2] = { (uint8_t)(base & 0xFF), (uint8_t)(base >> 8) };
		RegisterWriteBurst(Register::FEATURE_DATA_ADDR, addr, 2);
		RegisterWriteBurst(Register::FEATURE_DATA_TX, &array[i], 2);
		base += 1;
	}

	return true;
}

// Same sequence as bmi3_configure_enhanced_flexibility() of the Bosch SensorAPI
bool BMI323::LoadConfigArray()
{
	// 1. unlock the configuration page via the two command words, then select it in CFG_RES
	RegisterWrite(Register::CMD, CMD_CFG_UNLOCK_1);
	RegisterWrite(Register::CMD, CMD_CFG_UNLOCK_2);

	const uint16_t cfg_res = RegisterRead(Register::CFG_RES);

	if ((cfg_res >> 8) != CFG_RES_UNLOCKED_MSB) {
		PX4_DEBUG("CFG_RES unlock failed (0x%04x)", cfg_res);
		return false;
	}

	RegisterWrite(Register::CFG_RES, CFG_RES_UPLOAD_PAGE);

	// 2. code array, table array, then config version
	UploadConfigBlock(bmi323_config_array_code, sizeof(bmi323_config_array_code));
	UploadConfigBlock(bmi323_config_array_table, sizeof(bmi323_config_array_table));

	const uint8_t ver_addr[2] = { bmi323_config_version[0], bmi323_config_version[1] };
	RegisterWriteBurst(Register::FEATURE_DATA_ADDR, ver_addr, 2);
	RegisterWriteBurst(Register::FEATURE_DATA_TX, &bmi323_config_version[CFG_DATA_START], 2);

	// 3. return to the user register page
	RegisterWrite(Register::CFG_RES, 0x0000);

	// 4. feature engine reports a failed download in FEATURE_IO1.error_status
	const uint16_t status = RegisterRead(Register::FEATURE_IO1) & FEATURE_IO1_ERROR_STATUS_MASK;

	if (status == FEATURE_IO1_CONFIG_FAILED) {
		PX4_DEBUG("configuration string download failed (FEATURE_IO1 status 0x%x)", status);
		return false;
	}

	PX4_DEBUG("configuration array uploaded, FEATURE_IO1 status 0x%x", status);
	return true;
}

bool BMI323::Configure()
{
	// make sure no fatal error is pending before enabling the sensors
	if (RegisterRead(Register::ERR_REG) & ERR_REG_BIT::fatal_err) {
		PX4_DEBUG("fatal error flag set before configure");
		return false;
	}

	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// unsupported ACC/GYR configuration is flagged in ERR_REG
	const uint16_t err = RegisterRead(Register::ERR_REG);

	if (err & (ERR_REG_BIT::acc_conf_err | ERR_REG_BIT::gyr_conf_err | ERR_REG_BIT::fatal_err)) {
		PX4_DEBUG("ERR_REG 0x%04x after configure", err);
		success = false;
	}

	SetAccelScaleAndRange();
	SetGyroScale();

	return success;
}

bool BMI323::CheckErrors()
{
	const uint16_t err = RegisterRead(Register::ERR_REG);

	if (err & (ERR_REG_BIT::fatal_err | ERR_REG_BIT::acc_conf_err | ERR_REG_BIT::gyr_conf_err)) {
		PX4_DEBUG("ERR_REG 0x%04x", err);
		return false;
	}

	return true;
}

int BMI323::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<BMI323 *>(arg)->DataReady();
	return 0;
}

void BMI323::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool BMI323::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// INT1 is configured active-high, non-latched -> trigger on rising edge
	return px4_arch_gpiosetevent(_drdy_gpio, true, false, true, &DataReadyInterruptCallback, this) == 0;
}

bool BMI323::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool BMI323::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint16_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%04hX (0x%04hX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%04hX (0x%04hX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint16_t BMI323::RegisterRead(Register reg)
{
	// SPI read: [addr|0x80][dummy][LSB][MSB]
	RegRead16Buffer b{};
	b.cmd = static_cast<uint8_t>(reg) | DIR_READ;
	transfer((uint8_t *)&b, (uint8_t *)&b, sizeof(b));
	return (uint16_t)b.lsb | ((uint16_t)b.msb << 8);
}

void BMI323::RegisterWrite(Register reg, uint16_t value)
{
	uint8_t cmd[3] { (uint8_t)reg, (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
	transfer(cmd, cmd, sizeof(cmd));
}

void BMI323::RegisterSetAndClearBits(Register reg, uint16_t setbits, uint16_t clearbits)
{
	// read-modify-write, required because many registers contain reserved bits (datasheet chapter 6)
	const uint16_t orig_val = RegisterRead(reg);
	const uint16_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

// returns the FIFO fill level in 16 bit words
uint16_t BMI323::FIFOReadCount()
{
	CheckErrorRegister();

	RegRead16Buffer b{};
	b.cmd = static_cast<uint8_t>(Register::FIFO_FILL_LEVEL) | DIR_READ;

	if (transfer((uint8_t *)&b, (uint8_t *)&b, sizeof(b)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return (((uint16_t)b.msb << 8) | b.lsb) & 0x07FF; // 11 bit
}

// writes a gyro sample into the FIFO publish buffer (BMI323 axes -> PX4 FRD)
void BMI323::ProcessGyro(sensor_gyro_fifo_s *gyro, const FIFO::Data *frame)
{
	const uint8_t samples = gyro->samples;

	const int16_t gyro_x = combine(frame->x_msb, frame->x_lsb);
	const int16_t gyro_y = combine(frame->y_msb, frame->y_lsb);
	const int16_t gyro_z = combine(frame->z_msb, frame->z_lsb);

	gyro->x[samples] = gyro_x;
	gyro->y[samples] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
	gyro->z[samples] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;

	gyro->samples++;
}

// writes an accelerometer sample into the FIFO publish buffer (BMI323 axes -> PX4 FRD)
void BMI323::ProcessAccel(sensor_accel_fifo_s *accel, const FIFO::Data *frame)
{
	const uint8_t samples = accel->samples;

	const int16_t accel_x = combine(frame->x_msb, frame->x_lsb);
	const int16_t accel_y = combine(frame->y_msb, frame->y_lsb);
	const int16_t accel_z = combine(frame->z_msb, frame->z_lsb);

	accel->x[samples] = accel_x;
	accel->y[samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
	accel->z[samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;

	accel->samples++;
}

bool BMI323::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOReadBuffer buffer{};
	const size_t fifo_bytes = samples * sizeof(FIFO::Frame);

	// FIFO_DATA does not auto-increment: one burst read of cmd + dummy + whole frames.
	// Only complete frames are read; a partially read frame would be re-sent (datasheet 5.7.2).
	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, fifo_bytes + 2) != PX4_OK) {
		PX4_DEBUG("buffer transfer failed");
		perf_count(_bad_transfer_perf);
		return false;
	}

	sensor_accel_fifo_s accel_buffer{};
	accel_buffer.timestamp_sample = timestamp_sample;
	accel_buffer.dt = FIFO_SAMPLE_DT;

	sensor_gyro_fifo_s gyro_buffer{};
	gyro_buffer.timestamp_sample = timestamp_sample;
	gyro_buffer.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		const FIFO::Frame &frame = buffer.f[i];

		const uint16_t ax_raw = (uint16_t)combine(frame.accel.x_msb, frame.accel.x_lsb);
		const uint16_t gx_raw = (uint16_t)combine(frame.gyro.x_msb, frame.gyro.x_lsb);

		// settling frames after a config change: acc word0 = 0x7F01, gyro word0 = 0x7F02,
		// remaining words 0x8000 (datasheet table 18). 0x8000 alone = invalid sample.
		if (ax_raw == FIFO::DUMMY_ACC_WORD0 || gx_raw == FIFO::DUMMY_GYR_WORD0
		    || ax_raw == FIFO::INVALID || gx_raw == FIFO::INVALID) {
			PX4_DEBUG("dummy/invalid frame skipped");
			perf_count(_dummy_frame_perf);
			continue;
		}

		BMI323::ProcessAccel(&accel_buffer, &frame.accel);
		BMI323::ProcessGyro(&gyro_buffer, &frame.gyro);
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if ((accel_buffer.samples == 0) && (gyro_buffer.samples == 0)) {
		return false;
	}

	if (accel_buffer.samples > 0) {
		_px4_accel.updateFIFO(accel_buffer);
	}

	if (gyro_buffer.samples > 0) {
		_px4_gyro.updateFIFO(gyro_buffer);
	}

	return true;
}

void BMI323::FIFOReset()
{
	PX4_DEBUG("Resetting FIFO...");
	perf_count(_fifo_reset_perf);

	RegisterWrite(Register::FIFO_CTRL, FIFO_CTRL_FLUSH);

	_drdy_timestamp_sample.store(0);
}

void BMI323::UpdateTemperature()
{
	RegRead16Buffer b{};
	b.cmd = static_cast<uint8_t>(Register::TEMP_DATA) | DIR_READ;

	if (transfer((uint8_t *)&b, (uint8_t *)&b, sizeof(b)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const uint16_t raw = (uint16_t)b.lsb | ((uint16_t)b.msb << 8);

	float temperature = NAN;

	if (raw != 0x8000) { // 0x8000 = invalid
		temperature = 23.0f + (int16_t)raw / 512.0f; // 512 LSB/K, 0x0000 = 23 C
	}

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);
}
