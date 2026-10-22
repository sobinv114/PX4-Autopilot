/****************************************************************************
 *
 *   Copyright (c) 2024-2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file pwm_input_esp32.cpp
 *
 * PWM input for Espressif (ESP32-P4) using NuttX MCPWM capture.
 * Equivalent role to STM32/i.MXRT timer-capture pwm_input.
 *
 * Hardware path:
 *   GPIO → MCPWM capture (esp_mcpwm.c) → /dev/capX
 *   CAPIOC_FREQUENCY + CAPIOC_DUTYCYCLE → period/width µs → uORB pwm_input
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <math.h>

#include <nuttx/timers/capture.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/pwm_input.h>

#include <board_config.h>

#ifndef PWMIN_CAPTURE_DEVICE
#  define PWMIN_CAPTURE_DEVICE "/dev/cap0"
#endif

using namespace time_literals;

class PWMIN_ESP32 : public ModuleBase<PWMIN_ESP32>, public px4::ScheduledWorkItem
{
public:
	PWMIN_ESP32();
	~PWMIN_ESP32() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int init();

private:
	void Run() override;

	int _fd{-1};
	uORB::Publication<pwm_input_s> _pwm_input_pub{ORB_ID(pwm_input)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": sample")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comms_errors")};

	uint32_t _error_count{0};
};

PWMIN_ESP32::PWMIN_ESP32() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

PWMIN_ESP32::~PWMIN_ESP32()
{
	ScheduleClear();

	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
PWMIN_ESP32::init()
{
	_fd = ::open(PWMIN_CAPTURE_DEVICE, O_RDONLY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("open %s failed: %d (enable CONFIG_ESP_MCPWM_CAPTURE + register in bringup)",
			PWMIN_CAPTURE_DEVICE, errno);
		return -errno;
	}

	/* Start capture hardware (NuttX lower half start) */
	if (::ioctl(_fd, CAPIOC_ALL, 0) < 0) {
		/* CAPIOC_ALL may need struct; start via duty/freq still works after open on some builds.
		 * Ensure board bringup called esp_mcpwm_capture_initialize + cap_register.
		 */
	}

	ScheduleOnInterval(20_ms); /* 50 Hz sample, same idea as polling capture */
	return PX4_OK;
}

void
PWMIN_ESP32::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_fd < 0) {
		return;
	}

	perf_begin(_sample_perf);

	uint32_t freq_hz = 0;
	uint8_t duty_pct = 0;

	int ret_f = ::ioctl(_fd, CAPIOC_FREQUENCY, (unsigned long)&freq_hz);
	int ret_d = ::ioctl(_fd, CAPIOC_DUTYCYCLE, (unsigned long)&duty_pct);

	if (ret_f != 0 || ret_d != 0 || freq_hz == 0) {
		_error_count++;
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return;
	}

	/* Convert freq + duty% → period/width in microseconds (STM32 reports µs) */
	const float period_us_f = 1e6f / static_cast<float>(freq_hz);
	const float width_us_f  = period_us_f * (static_cast<float>(duty_pct) / 100.0f);

	pwm_input_s report{};
	report.timestamp   = hrt_absolute_time();
	report.error_count = _error_count;
	report.period      = static_cast<uint32_t>(lroundf(period_us_f));
	report.pulse_width = static_cast<uint32_t>(lroundf(width_us_f));

	_pwm_input_pub.publish(report);

	perf_end(_sample_perf);
}

int
PWMIN_ESP32::task_spawn(int argc, char *argv[])
{
	PWMIN_ESP32 *inst = new PWMIN_ESP32();

	if (!inst) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(inst);
	_task_id = task_id_is_work_queue;

	if (inst->init() != PX4_OK) {
		delete inst;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
PWMIN_ESP32::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
PWMIN_ESP32::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
PWM input on Espressif using NuttX MCPWM capture (/dev/capX).
Publishes the uORB pwm_input topic (period and pulse width in microseconds).
Requires CONFIG_ESP_MCPWM_CAPTURE and a registered capture device.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_input", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int pwm_input_main(int argc, char *argv[])
{
	return PWMIN_ESP32::main(argc, argv);
}
