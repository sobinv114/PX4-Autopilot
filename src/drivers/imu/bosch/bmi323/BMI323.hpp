/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * (BSD-3-Clause, see Bosch_BMI323_registers.hpp for full text)
 *
 ****************************************************************************/

/**
 * @file BMI323.hpp
 *
 * Driver for the Bosch BMI323 connected via SPI.
 */

#pragma once

#include "Bosch_BMI323_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

// Upload the Bosch configuration array (enable feature engine + "enhanced flexibility" config).
// Not needed for accel/gyro/FIFO; set to 0 to skip it.
#ifndef BMI323_LOAD_CONFIG_FILE
#define BMI323_LOAD_CONFIG_FILE 1
#endif

using namespace Bosch_BMI323;

class BMI323 : public device::SPI, public I2CSPIDriver<BMI323>
{
public:
	BMI323(const I2CSPIDriverConfig &config);
	~BMI323() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr uint32_t RATE{1600}; // accel + gyro ODR in Hz
	static constexpr float FIFO_SAMPLE_DT{1e6f / RATE};

	static constexpr int32_t FIFO_MAX_SAMPLES{
		math::min(FIFO::SIZE_WORDS / FIFO::FRAME_WORDS,
			  sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]))};

	// SPI read of FIFO_FILL_LEVEL: cmd, dummy, LSB, MSB
	struct RegRead16Buffer {
		uint8_t cmd;
		uint8_t dummy{0};
		uint8_t lsb{0};
		uint8_t msb{0};
	};

	struct FIFOReadBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_DATA) | DIR_READ};
		uint8_t dummy{0};
		FIFO::Frame f[FIFO_MAX_SAMPLES] {};
	};

	// ensure no struct padding
	static_assert(sizeof(FIFOReadBuffer) == (2 + FIFO_MAX_SAMPLES * sizeof(FIFO::Frame)));

	struct register_config_t {
		Register reg;
		uint16_t set_bits{0};
		uint16_t clear_bits{0};
	};

	int probe() override;

	bool Reset();
	bool EnableFeatureEngineStart();
	bool LoadConfigArray();
	bool UploadConfigBlock(const uint8_t *array, size_t size);
	void RegisterWriteBurst(Register reg, const uint8_t *data, size_t len);
	bool Configure();
	bool CheckErrors();
	void CheckErrorRegister(); // debug decoder of ERR_REG (like the BMI270 driver)

	void SetAccelScaleAndRange();
	void SetGyroScale();

	void ProcessGyro(sensor_gyro_fifo_s *gyro, const FIFO::Data *gyro_frame);
	void ProcessAccel(sensor_accel_fifo_s *accel, const FIFO::Data *accel_frame);

	void ConfigureSampleRate(int sample_rate = 0);
	void ConfigureFIFOWatermark(uint8_t samples);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint16_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint16_t value);
	void RegisterSetAndClearBits(Register reg, uint16_t setbits, uint16_t clearbits);

	uint16_t FIFOReadCount(); // words
	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	void UpdateTemperature();

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _dummy_frame_perf{perf_alloc(PC_COUNT, MODULE_NAME": dummy frame")};

	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _feature_engine_timestamp{0};
	bool _feature_engine_active{false};
	bool _config_array_loaded{false};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		WAIT_FOR_FEATURE_ENGINE,
		CONFIGURE,
		WAIT_FOR_STARTUP,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	int32_t _fifo_samples{2};

	// ACC/GYR: high performance mode, ODR/2 bandwidth, no averaging
	static constexpr uint16_t ACC_CONF_VALUE{CONF::MODE_HIGH_PERF | CONF::BW_ODR_2 | ACC_RANGE::G16 | CONF::ODR_1600};
	static constexpr uint16_t GYR_CONF_VALUE{CONF::MODE_HIGH_PERF | CONF::BW_ODR_2 | GYR_RANGE::DPS2000 | CONF::ODR_1600};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{7};

	// Order matters: the FIFO must be configured BEFORE the sensors are enabled (datasheet 5.7.2),
	// so FIFO_CONF comes first and ACC_CONF / GYR_CONF last.
	register_config_t _register_cfg[size_register_cfg] {
		// Register                  | Set bits, Clear bits
		{ Register::FIFO_CONF,      FIFO_CONF_BIT::fifo_acc_en | FIFO_CONF_BIT::fifo_gyr_en,
		  FIFO_CONF_BIT::fifo_stop_on_full | FIFO_CONF_BIT::fifo_time_en | FIFO_CONF_BIT::fifo_temp_en },
		{ Register::FIFO_WATERMARK, 0, 0 }, // filled in by ConfigureFIFOWatermark()
		{ Register::IO_INT_CTRL,    IO_INT_CTRL_BIT::int1_lvl_high | IO_INT_CTRL_BIT::int1_output_en, IO_INT_CTRL_BIT::int1_od },
		{ Register::INT_CONF,       0, INT_CONF_LATCH },
		{ Register::INT_MAP2,       INT_MAP2_BIT::fifo_wm_int1, static_cast<uint16_t>(INT_MAP2_BIT::drdy_fifo_fields_mask & ~INT_MAP2_BIT::fifo_wm_int1) },
		{ Register::GYR_CONF,       GYR_CONF_VALUE, static_cast<uint16_t>(CONF::MASK & ~GYR_CONF_VALUE) },
		{ Register::ACC_CONF,       ACC_CONF_VALUE, static_cast<uint16_t>(CONF::MASK & ~ACC_CONF_VALUE) },
	};
};
