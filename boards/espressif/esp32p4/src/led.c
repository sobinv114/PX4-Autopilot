// /****************************************************************************
//  *
//  *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  * 3. Neither the name PX4 nor the names of its contributors may be
//  *    used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/


// Each status gets a group of 4 pixels: NEOPIXEL 16BIT
// Logical LED	Pixels	        Colour
// LED_RED	0 to 3      	red
// LED_GREEN	4 to 7	        green
// LED_BLUE	8 to 11	        blue
// LED_SAFETY	12 to 15	white

/**
 * @file led.c
 * Board LED hooks for ESP32-P4 on a WS2812 strip. Used for boot indication
 * only; once the neopixel driver starts it calls board_led_release() and
 * these hooks become no-ops.
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>

#include "board_config.h"
#include <drivers/drv_board_led.h>

#define LED_DEV    "/dev/leds0"
#define LED_LEVEL  8                 /* 0..255 brightness */
#define GROUP_SIZE 4

#ifndef BOARD_HAS_N_S_RGB_LED
#  define BOARD_HAS_N_S_RGB_LED 16
#endif

__BEGIN_DECLS
__EXPORT void led_init(void);
__EXPORT void led_on(int led);
__EXPORT void led_off(int led);
__EXPORT void led_toggle(int led);
__EXPORT void board_led_release(void);
__END_DECLS

static bool g_state[4];              /* red, green, blue, safety */
static volatile bool g_released;

static int group_of(int led)
{
	switch (led) {
	case LED_RED:    return 0;       /* LED_AMBER has the same value */
	case LED_GREEN:  return 1;
	case LED_BLUE:   return 2;
	case LED_SAFETY: return 3;
	default:         return -1;
	}
}

static uint32_t group_colour(int g)
{
	const uint32_t v = LED_LEVEL;

	switch (g) {
	case 0:  return v << 16;                       /* 0x00RRGGBB */
	case 1:  return v << 8;
	case 2:  return v;
	default: return (v << 16) | (v << 8) | v;
	}
}

static void led_push(void)
{
	if (g_released) {
		return;
	}

	uint32_t buf[BOARD_HAS_N_S_RGB_LED];

	for (int i = 0; i < BOARD_HAS_N_S_RGB_LED; i++) {
		const int g = i / GROUP_SIZE;
		buf[i] = (g < 4 && g_state[g]) ? group_colour(g) : 0;
	}

	/* open per call: file descriptors are not shared between tasks */
	int fd = open(LED_DEV, O_WRONLY);

	if (fd < 0) {
		return;
	}

	(void)write(fd, buf, sizeof(buf));
	close(fd);
}

__EXPORT void led_init(void) { }

__EXPORT void led_on(int led)
{
	const int g = group_of(led);

	if (g >= 0) {
		g_state[g] = true;
		led_push();
	}
}

__EXPORT void led_off(int led)
{
	const int g = group_of(led);

	if (g >= 0) {
		g_state[g] = false;
		led_push();
	}
}

__EXPORT void led_toggle(int led)
{
	const int g = group_of(led);

	if (g >= 0) {
		g_state[g] = !g_state[g];
		led_push();
	}
}

/* Called by the neopixel driver when it takes over the strip */
__EXPORT void board_led_release(void)
{
	g_released = true;
}
