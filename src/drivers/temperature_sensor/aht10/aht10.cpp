/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "aht10.h"

AHT10::AHT10(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
	// Set device ID once during initialization
	_sensor_temp.device_id = get_device_id();
	_sensor_hygro.device_id = get_device_id();
}

AHT10::~AHT10()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
}

int AHT10::probe()
{
	uint8_t reset_cmd = AHT10_CMD_SOFT_RESET;
	int ret = transfer(&reset_cmd, 1, nullptr, 0);

	if (ret != PX4_OK) {
		return ret;
	}

	px4_usleep(25000); // 25ms recovery after reset

	uint8_t status = 0;
	ret = transfer(nullptr, 0, &status, 1);

	if (ret != PX4_OK) {
		return ret;
	}

	if (!(status & AHT10_STATUS_CALIBRATED)) {
		uint8_t init_cmd[3] = {AHT10_CMD_INIT, 0x08, 0x00};
		transfer(init_cmd, sizeof(init_cmd), nullptr, 0);
		px4_usleep(20000);
	}

	PX4_INFO("AHT10 found at address 0x%02X (status: 0x%02X)", get_device_address(), status);
	return PX4_OK;
}


int AHT10::init()
{
	int ret = I2C::init();
	if (ret != PX4_OK) {
		PX4_ERR("AHT10 I2C init failed");
		return ret;
	}

	if (!_sensor_temp_pub.advertise()) {
		PX4_ERR("sensor_temp advertise failed");
		return PX4_ERROR;
	}
	if (!_sensor_hygro_pub.advertise()) {
		PX4_ERR("sensor_hygrometer advertise failed");
		return PX4_ERROR;
	}

	PX4_INFO("sensor_temp and sensor_hygrometer advertised successfully");

	ScheduleNow();
	return PX4_OK;
}

int AHT10::trigger_measurement()
{
	uint8_t cmd[3] = {AHT10_CMD_TRIGGER_MEASURE, 0x33, 0x00};
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}




// int AHT10::read_and_publish()
// {
// 	uint8_t buf[6] = {};
// 	int ret = transfer(nullptr, 0, buf, sizeof(buf));

// 	if (ret != PX4_OK) {
// 		return ret;
// 	}

// 	if (buf[0] & AHT10_STATUS_BUSY) {
// 		return PX4_ERROR;
// 	}

// 	uint32_t raw_temperature = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];
// 	float temperature = (((float)raw_temperature / 1048576.0f) * 200.0f) - 50.0f;

// 	_sensor_temp.timestamp = hrt_absolute_time();
// 	_sensor_temp.temperature = temperature;
// 	// device_id is already set in the constructor

// 	bool published = _sensor_temp_pub.publish(_sensor_temp);

// 	// Temporary debug
// 	static int counter = 0;
// 	if (counter++ % 10 == 0) {
// 		PX4_INFO("AHT10 publish: %s  temp=%.2f  ts=%llu  device_id=%" PRIu32,
// 			published ? "OK" : "FAILED",
// 			(double)temperature,
// 			(unsigned long long)_sensor_temp.timestamp,
// 			_sensor_temp.device_id);
// 	}

// 	return published ? PX4_OK : PX4_ERROR;
// }


int AHT10::read_and_publish()
{
	uint8_t buf[6] = {};
	const hrt_abstime t_sample = hrt_absolute_time();
	int ret = transfer(nullptr, 0, buf, sizeof(buf));

	if (ret != PX4_OK) {
		return ret;
	}

	if (buf[0] & AHT10_STATUS_BUSY) {
		return PX4_ERROR;
	}

	// 20-bit humidity: buf[1], buf[2], upper nibble of buf[3]
	const uint32_t raw_humidity = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | ((uint32_t)buf[3] >> 4);
	// 20-bit temperature: lower nibble of buf[3], buf[4], buf[5]
	const uint32_t raw_temperature = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];

	float humidity = ((float)raw_humidity / 1048576.0f) * 100.0f;
	const float temperature = (((float)raw_temperature / 1048576.0f) * 200.0f) - 50.0f;

	// clamp to the physical range
	if (humidity < 0.f) { humidity = 0.f; }
	if (humidity > 100.f) { humidity = 100.f; }

	const hrt_abstime now = hrt_absolute_time();

	_sensor_temp.temperature = temperature;
	_sensor_temp.timestamp = now;
	_sensor_temp_pub.publish(_sensor_temp);

	_sensor_hygro.timestamp = now;
	_sensor_hygro.timestamp_sample = t_sample;
	_sensor_hygro.temperature = temperature;
	_sensor_hygro.humidity = humidity;
	_sensor_hygro_pub.publish(_sensor_hygro);

	return PX4_OK;
}


void AHT10::RunImpl()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	switch (_state) {
	case State::Trigger: {
		if (trigger_measurement() == PX4_OK) {
			_state = State::Read;
			// Sensor needs > 75ms to convert
			ScheduleDelayed(80_ms);
		} else {
			perf_count(_comms_errors);
			ScheduleDelayed(200_ms);
		}
		break;
	}

	case State::Read: {
		if (read_and_publish() != PX4_OK) {
			perf_count(_comms_errors);
		}

		_state = State::Trigger;
		// Wait 120ms to make the total cycle 200ms (5Hz), matching MCP9808
		ScheduleDelayed(120_ms);
		break;
	}
	}

	perf_end(_cycle_perf);
}
