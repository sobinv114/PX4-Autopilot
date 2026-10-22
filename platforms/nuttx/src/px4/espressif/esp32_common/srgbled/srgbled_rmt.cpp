/****************************************************************************
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 ****************************************************************************/
/**
 * @file srgbled_rmt.cpp
 * Neopixel/WS2812 backend for ESP32-P4 using NuttX RMT and WS2812 driver.
 *
 * The board must define:
 *   BOARD_HAS_N_S_RGB_LED  - number of LEDs
 *   BOARD_S_RGB_LED_GPIO   - GPIO wired to DIN
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <parameters/param.h>
#include <board_config.h>
#include <drivers/drv_neopixel.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

#define private _private
#include <nuttx/leds/ws2812.h>
#undef private

#if defined(BOARD_HAS_N_S_RGB_LED)

/* Fallback definitions */
#ifndef BOARD_S_RGB_LED_GPIO
#  error "BOARD_S_RGB_LED_GPIO must be defined in board_config.h"
#endif

#define LED_BRIGHTNESS S_RGB_LED_BRIGHTNESS   /* 1..255, 255 = full */

#ifndef S_RGB_LED_RMT_CHANNEL
#  define S_RGB_LED_RMT_CHANNEL 0
#endif

#define WS2812_DEV_PATH "/dev/leds0"

static int g_fd = -1;

/* Declare the NuttX ESP32 lower-half bringup function */
extern "C" {
int esp_ws2812_setup(const char *devpath, int channel, int gpio, uint16_t nleds);
}

extern "C" void board_led_release(void);


// int neopixel_init(neopixel::NeoLEDData *led_data, int number_of_packages)
// {
// 	if (number_of_packages > BOARD_HAS_N_S_RGB_LED) {
// 		return -1;
// 	}

// 	if (g_fd >= 0) {
// 		return 0; // Already open
// 	}

// 	// 1. Try opening the driver if it was already initialized
// 	g_fd = open(WS2812_DEV_PATH, O_WRONLY);

// 	// 2. If not registered yet, initialize the RMT + WS2812 lower-half on the fly
// 	if (g_fd < 0) {
// 		int ret = esp_ws2812_setup(WS2812_DEV_PATH,
// 					   S_RGB_LED_RMT_CHANNEL,
// 					   BOARD_S_RGB_LED_GPIO,
// 					   BOARD_HAS_N_S_RGB_LED);

// 		if (ret < 0) {
// 			PX4_ERR("Failed to initialize ESP32-P4 RMT WS2812 (ret: %d)", ret);
// 			return ret;
// 		}

// 		g_fd = open(WS2812_DEV_PATH, O_WRONLY);

// 		if (g_fd < 0) {
// 			PX4_ERR("Failed to open %s after setup: %d", WS2812_DEV_PATH, errno);
// 			return -1;
// 		}
// 	}

// 	{
// 		uint32_t test = 0x00200000;   /* dim red, 0x00RRGGBB */
// 		ssize_t n = write(g_fd, &test, sizeof(test));
// 		PX4_INFO("neo: init test write=%d errno=%d", (int)n, errno);
// 	}

// 	return 0;
// }

static uint32_t get_brightness()
{
	int32_t v = S_RGB_LED_BRIGHTNESS;  /* default from board */

	param_t h = param_find("WS2812_BRIGHT");

	if (h != PARAM_INVALID) {
		param_get(h, &v);
	}

	if (v < 1)   { v = 1; }
	if (v > 255) { v = 255; }

	return (uint32_t)v;
}

int neopixel_init(neopixel::NeoLEDData *led_data, int number_of_packages)
{
	if (number_of_packages <= 0 || number_of_packages > BOARD_HAS_N_S_RGB_LED) {
		return -1;
	}

	board_led_release();   /* boot indication ends, driver owns the strip */

	return 0;   /* /dev/leds0 is created in board init (init.c) */
}



extern int neopixel_write(neopixel::NeoLEDData *led_data, int number_of_packages)
{
	if (number_of_packages <= 0 || number_of_packages > BOARD_HAS_N_S_RGB_LED) {
		return -1;
	}

	/* 1. Get the current brightness from the QGC parameter */
	const uint32_t bright = get_brightness();


	uint32_t pix[BOARD_HAS_N_S_RGB_LED];

	// for (int i = 0; i < number_of_packages; i++) {
	// 	const uint32_t l = led_data[i].data.l;
	// 	const uint32_t g = (l >> 16) & 0xff;
	// 	const uint32_t r = (l >> 8)  & 0xff;
	// 	const uint32_t b = l & 0xff;
	// 	pix[i] = (r << 16) | (g << 8) | b;
	// }

	for (int i = 0; i < number_of_packages; i++) {
		const uint32_t l = led_data[i].data.l;
		uint32_t g = (l >> 16) & 0xff;
		uint32_t r = (l >> 8)  & 0xff;
		uint32_t b = l & 0xff;

		r = (r * bright) / 255;
		g = (g * bright) / 255;
		b = (b * bright) / 255;

		pix[i] = (r << 16) | (g << 8) | b;
	}

	const size_t bytes = number_of_packages * sizeof(uint32_t);

	static uint32_t last[BOARD_HAS_N_S_RGB_LED];
	static int last_n = 0;


	/* 3. memcmp compares the SCALED values.
	 * If you change LED_BRIGHT in QGC, pix[] values change,
	 * memcmp detects the difference, and it updates immediately!
	 */
	if (last_n == number_of_packages && memcmp(last, pix, bytes) == 0) {
		return 0;
	}

	/* open in THIS task: fds are not shared between task groups */
	int fd = open(WS2812_DEV_PATH, O_WRONLY);

	if (fd < 0) {
		return -1;
	}

	ssize_t written = write(fd, pix, bytes);
	close(fd);

	if (written != (ssize_t)bytes) {
		return -1;
	}

	memcpy(last, pix, bytes);
	last_n = number_of_packages;
	return 0;
}


int neopixel_deinit()
{
	if (g_fd >= 0) {
		close(g_fd);
		g_fd = -1;
	}

	return OK;
}
#endif // BOARD_HAS_N_S_RGB_LED
