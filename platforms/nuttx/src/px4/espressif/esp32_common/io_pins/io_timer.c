/****************************************************************************
 *
 *   Copyright (C) 2021-2026 PX4 Development Team. All rights reserved.
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
 * @file io_timer.c
 *
 * Servo driver supporting PWM servos connected to ESP32-P4 LEDC channels.
 */

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <nuttx/queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer.h>

/* Compatibility mapping for LEDC Timer 0 channels */
#if defined(CONFIG_ESPRESSIF_LEDC_TIMER0_CHANNELS)
#  define P4_LEDC_TIM0_CHANNELS CONFIG_ESPRESSIF_LEDC_TIMER0_CHANNELS
#elif defined(CONFIG_ESP32P4_LEDC_TIM0_CHANNELS)
#  define P4_LEDC_TIM0_CHANNELS CONFIG_ESP32P4_LEDC_TIM0_CHANNELS
#else
#  define P4_LEDC_TIM0_CHANNELS 0
#endif

/* Compatibility mapping for LEDC Timer 1 channels */
#if defined(CONFIG_ESPRESSIF_LEDC_TIMER1_CHANNELS)
#  define P4_LEDC_TIM1_CHANNELS CONFIG_ESPRESSIF_LEDC_TIMER1_CHANNELS
#elif defined(CONFIG_ESP32P4_LEDC_TIM1_CHANNELS)
#  define P4_LEDC_TIM1_CHANNELS CONFIG_ESP32P4_LEDC_TIM1_CHANNELS
#else
#  define P4_LEDC_TIM1_CHANNELS 0
#endif

uint32_t io_timer_get_group(unsigned timer)
{
	uint32_t channel_mask = 0;

	if (timer == 0) {
		for (int i = 0; i < P4_LEDC_TIM0_CHANNELS; i++) {
			channel_mask |= (1 << i);
		}
		return channel_mask;
	}
#if (P4_LEDC_TIM1_CHANNELS > 0)
	else if (timer == 1) {
		for (int i = 0; i < P4_LEDC_TIM1_CHANNELS; i++) {
			channel_mask |= (1 << (i + P4_LEDC_TIM0_CHANNELS));
		}
		return channel_mask;
	}
#endif

	return 0;
}
