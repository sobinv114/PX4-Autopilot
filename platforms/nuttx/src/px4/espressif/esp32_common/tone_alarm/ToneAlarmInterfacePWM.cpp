/****************************************************************************
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 ****************************************************************************/
#include <px4_platform_common/log.h>
#include <drivers/drv_tone_alarm.h>
#include <parameters/param.h>
#include <px4_platform_common/defines.h>
#include <board_config.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/tune_control.h>

#include <nuttx/timers/pwm.h>

extern "C" {
#include "espressif/esp_ledc.h"
}

/* Fallback definitions if not defined in board_config.h */
#ifndef TONE_ALARM_LEDC_TIMER
#  define TONE_ALARM_LEDC_TIMER 2
#endif

#ifndef TONE_ALARM_LEDC_CHANNEL
#  define TONE_ALARM_LEDC_CHANNEL 2
#endif

#define TONE_REF_VOLUME 90   // tune_control default volume = "full" loudness

namespace ToneAlarmInterface
{

static struct pwm_lowerhalf_s *g_pwm = nullptr;
static uORB::Subscription g_tune_sub{ORB_ID(tune_control)};

void init()
{
	if (g_pwm == nullptr) {
		struct pwm_lowerhalf_s *pwm = esp_ledc_init(TONE_ALARM_LEDC_TIMER);

		if (pwm != nullptr && pwm->ops->setup != nullptr && pwm->ops->setup(pwm) == OK) {
			g_pwm = pwm;   // pin routing and idle-low handled by esp_ledc.c
		}
	}
}

// hrt_abstime start_note(unsigned frequency)
// {
// 	PX4_INFO("tone: start freq=%u", frequency);
// 	if (frequency == 0) {
// 		stop_note();
// 		return hrt_absolute_time();
// 	}

// 	hrt_abstime time_started = hrt_absolute_time();

// 	if (g_pwm != nullptr && frequency > 0) {
// 		struct pwm_info_s info {};
// 		info.frequency = frequency;
// 		info.channels[0].channel = 0;       // ignored by esp_ledc.c, index 0 is used
// 		info.channels[0].duty    = 0x8000;  // 50%

// 		if (g_pwm->ops->start(g_pwm, &info) != OK) {
// 			PX4_ERR("tone start failed (freq %u)", frequency);
// 		}
// 	}

// 	return time_started;
// }


static uint32_t get_master_volume()
{
	static param_t h = param_find("TONE_VOLUME");
	int32_t v = 100;

	if (h != PARAM_INVALID) {
		param_get(h, &v);
	}

	if (v < 0)   { v = 0; }
	if (v > 100) { v = 100; }

	return (uint32_t)v;
}

void stop_note()
{
	if (g_pwm != nullptr) {
		g_pwm->ops->stop(g_pwm);
	}
}

hrt_abstime start_note(unsigned frequency)
{
	const hrt_abstime time_started = hrt_absolute_time();

	if (g_pwm == nullptr) {
		return time_started;
	}

	if (frequency == 0) {          // rest
		stop_note();
		return time_started;
	}

	// volume from the last tune_control message
	uint32_t volume = TONE_REF_VOLUME;
	tune_control_s tc{};

	if (g_tune_sub.copy(&tc)) {
		volume = tc.volume;
	}

	// normalise: default tune volume -> 100%, anything above is clamped
	volume = (volume * 100) / TONE_REF_VOLUME;

	if (volume > 100) { volume = 100; }

	// QGC master volume 0..100
	volume = (volume * get_master_volume()) / 100;

	if (volume == 0) {
		stop_note();
		return time_started;
	}

	// volume 1..100 -> amplitude 0..1 (linear, so 100 = full 50% duty)
	const float a = (float)volume / 100.0f;

	// square-wave fundamental amplitude is sin(pi*D), so D = asin(a)/pi (max 0.5)
	float d = asinf(a) / 3.14159265f;

	if (d < 0.004f) {
		d = 0.004f;
	}

	struct pwm_info_s info {};
	info.frequency = frequency;
	info.channels[0].channel = 0;
	info.channels[0].duty    = (uint16_t)((d)*2 * 65535.0f);

	if (g_pwm->ops->start(g_pwm, &info) != OK) {
		PX4_ERR("tone start failed (freq %u)", frequency);
	}

	return time_started;
}

} /* namespace ToneAlarmInterface */
