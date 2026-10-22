// /****************************************************************************
//  *
//  *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
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

// /**
//  * @file init.c
//  *
//  * ESP32-P4 early startup code. This file implements board_app_initialize()
//  * called early by NSH during boot before the rcS startup script runs.
//  */


#include "nuttx/config.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <board_config.h>
#include <px4_platform_common/log.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>
#include <unistd.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mtd/mtd.h>
#include "board_config.h"
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/px4_macros.h>
#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>
#include "esp_private/esp_clk.h"
#include "esp32p4-function-ev-board.h"
#include "espressif/esp_gpio.h"
#include "espressif/esp_spi.h"
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/mm/gran.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_arch/io_timer.h>
#include <px4_platform/board_determine_hw_info.h>
#include <px4_platform/board_dma_alloc.h>


#if defined(CONFIG_FS_ROMFS)
#include <nuttx/drivers/ramdisk.h>
#include <sys/mount.h>

extern const unsigned char romfs_img[];
extern const unsigned int romfs_img_len;
#endif


#if defined(CONFIG_ESPRESSIF_SPIFLASH) || defined(CONFIG_ESP32P4_SPIFLASH)
#  include "espressif/esp_spiflash_mtd.h"
#  include "espressif/esp_spiflash.h"
#endif

#if defined(CONFIG_ESPRESSIF_LEDC) || defined(CONFIG_PWM)
#  include "espressif/esp_ledc.h"
#endif


#if defined(CONFIG_ESP_RMT) && defined(CONFIG_WS2812)
#  include "espressif/esp_rmt.h"
#  include "espressif/esp_ws2812.h"
#endif

#if defined(CONFIG_TIMER) || defined(CONFIG_ONESHOT)
#  include "esp32p4_board_tim.h"
#endif

#if defined(CONFIG_ESP_MCPWM_CAPTURE)
#  include "espressif/esp_mcpwm.h"
#  include <nuttx/timers/capture.h>
#endif

#ifndef CONFIG_ESPRESSIF_STORAGE_MTD_OFFSET
#  define CONFIG_ESPRESSIF_STORAGE_MTD_OFFSET 0x310000
#endif

#ifndef CONFIG_ESPRESSIF_STORAGE_MTD_SIZE
#  define CONFIG_ESPRESSIF_STORAGE_MTD_SIZE   0x20000
#endif


#ifndef putreg32
#  define putreg32(v, a) (*(volatile uint32_t *)(a) = (uint32_t)(v))
#endif

// extern int esp_rom_printf(const char *fmt, ...);

/* Function prototypes */
int can_devinit(void);

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void esp32p4_spiinitialize(void);
extern void lib_cxx_initialize(void);
extern void sd_check(const char *tag);
__END_DECLS

__EXPORT void board_peripheral_reset(int ms)
{
	if (ms > 0) usleep(ms * 1000);
}

__EXPORT void board_on_reset(int status) {}

__EXPORT void esp32p4_board_initialize(void)
{
	board_on_reset(-1);
}

__EXPORT void esp_board_initialize(void)
{
	esp32p4_board_initialize();
}

int board_read_VBUS_state(void)
{
	return 1;
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
	int ret = OK;

	#ifdef CONFIG_MMCSD
	       ret = esp32p4_sdio_initialize();
		if (ret != OK) {
			syslog(LOG_ERR, "Failed to initialize SDMMC\n");
			/* Do NOT return ret here; we still want the autopilot to boot
			even if the user forgot to insert an SD card! */
		}
		syslog(LOG_ERR, "Successfully initialize SDMMC\n");
	#endif
        usleep(10000);


	syslog(LOG_INFO, "\n[boot] CPU SPEED: %d MHz\n", esp_clk_cpu_freq() / 1000000);

	// /* RUN C++ GLOBAL CONSTRUCTORS EXACTLY ONCE! */
	syslog(LOG_INFO, "[boot] Running C++ global constructors\n");
	lib_cxx_initialize();
	syslog(LOG_INFO, "[boot] C++ global constructors DONE\n");

	usleep(10000);

        /* NOW IT IS SAFE TO START PX4 */
	syslog(LOG_INFO, "PX4 PLATFORM INIT PREPARE\n");
	px4_platform_init();
  	syslog(LOG_INFO, "PX4 PLATFORM INIT OK\n");

	usleep(10000);

	if (OK == board_determine_hw_info()) {
		syslog(LOG_INFO, "[boot] Rev 0x%1x : Ver 0x%1x %s\n", board_get_hw_revision(), board_get_hw_version(),
		       board_get_hw_type_name());

	} else {
		syslog(LOG_ERR, "[boot] Failed to read HW revision and version\n");
	}

	// led_init();
	// led_on(LED_RED);
	// led_on(LED_GREEN);

	// drv_led_start();


	esp32p4_spiinitialize();

#if defined(CONFIG_ESPRESSIF_SPI2) || defined(CONFIG_ESP32P4_SPI2)
	struct spi_dev_s *spi2 = esp_spibus_initialize(2);
	if (spi2) {
		SPI_SETFREQUENCY(spi2, 20000000);
		SPI_SETBITS(spi2, 8);
		SPI_SETMODE(spi2, SPIDEV_MODE3);
		syslog(LOG_INFO, "[boot] >>> SPI2 enabled <<<\n");
	}
#endif

#if defined(CONFIG_ESPRESSIF_SPI3) || defined(CONFIG_ESP32P4_SPI3)
	struct spi_dev_s *spi3 = esp_spibus_initialize(3);
	if (spi3) {
		SPI_SETFREQUENCY(spi3, 20000000);
		SPI_SETBITS(spi3, 8);
		SPI_SETMODE(spi3, SPIDEV_MODE3);
		syslog(LOG_INFO, "[boot] >>> SPI3 enabled <<<\n");
	}
#endif

#if defined(CONFIG_TIMER) || defined(CONFIG_ONESHOT)
	board_tim_init();
#endif

	usleep(10000);

#if defined(CONFIG_ESPRESSIF_SPIFLASH) || defined(CONFIG_ESP32P4_SPIFLASH)
	FAR struct mtd_dev_s *mtd = esp_spiflash_alloc_mtdpart(
		CONFIG_ESPRESSIF_STORAGE_MTD_OFFSET,
		CONFIG_ESPRESSIF_STORAGE_MTD_SIZE);

	if (!mtd) {
		syslog(LOG_ERR, "ERROR: Failed to allocate SPI Flash MTD partition\n");
	} else {
		ret = register_mtddriver("/fs/mtd_params", mtd, 0777, NULL);
		if (ret < 0) {
			syslog(LOG_ERR, "ERROR: Failed to register /fs/mtd_params: %d\n", ret);
		} else {
			syslog(LOG_INFO, "[boot] MTD partition mounted at /fs/mtd_params\n");
		}
	}
#endif


#if defined(CONFIG_CAN)
    syslog(LOG_INFO, "TWAI CAN Starting\n");
    can_devinit();
    syslog(LOG_INFO, "TWAI CAN Started\n");
#endif


	usleep(10000);

#if defined(CONFIG_ESP_RMT) && defined(CONFIG_WS2812) && defined(BOARD_HAS_N_S_RGB_LED)
        usleep(5000);
	struct rmt_dev_s *rmt = esp_rmt_tx_init(BOARD_S_RGB_LED_GPIO);
	if (rmt == NULL) {
		syslog(LOG_ERR, "[boot] RMT TX init failed\n");
	} else if (esp_ws2812_setup("/dev/leds0", rmt, BOARD_HAS_N_S_RGB_LED, false) == NULL) {
		syslog(LOG_ERR, "[boot] ws2812 setup failed\n");
	} else {
		syslog(LOG_INFO, "[boot] ws2812 /dev/leds0 ready\n");
	}
#endif

/* LEDs now map to the single WS2812 pixel (needs /dev/leds0) */
/* after the ws2812 setup block */
	drv_led_start();
	led_on(LED_RED);            /* board init running */

/* MCPWM PWM capture → /dev/capX for PX4 pwm_input_esp32 */
#  if defined(CONFIG_ESP_MCPWM_CAPTURE_CH0)
  {
    /* channel 0, GPIO from Kconfig or board pin */
    const int pin0 =
#    ifdef CONFIG_ESP_MCPWM_CAPTURE_CH0_GPIO
      CONFIG_ESP_MCPWM_CAPTURE_CH0_GPIO;
#    else
      19;  /* pick a free GPIO if Kconfig has no pin */
#    endif

    struct cap_lowerhalf_s *cap0 = esp_mcpwm_capture_initialize(0, pin0);

    if (cap0 == NULL) {
      syslog(LOG_ERR, "[boot] MCPWM capture CH0 init failed\n");
    } else if (cap_register("/dev/cap0", cap0) < 0) {
      syslog(LOG_ERR, "[boot] cap_register /dev/cap0 failed\n");
    } else {
      syslog(LOG_INFO, "[boot] MCPWM capture /dev/cap0 on GPIO %d\n", pin0);
    }
  }
#  endif

#  if defined(CONFIG_ESP_MCPWM_CAPTURE_CH1)
  {
    const int pin1 =
#    ifdef CONFIG_ESP_MCPWM_CAPTURE_CH1_GPIO
      CONFIG_ESP_MCPWM_CAPTURE_CH1_GPIO;
#    else
      19;
#    endif

    struct cap_lowerhalf_s *cap1 = esp_mcpwm_capture_initialize(1, pin1);

    if (cap1 == NULL) {
      syslog(LOG_ERR, "[boot] MCPWM capture CH1 init failed\n");
    } else if (cap_register("/dev/cap1", cap1) < 0) {
      syslog(LOG_ERR, "[boot] cap_register /dev/cap1 failed\n");
    } else {
      syslog(LOG_INFO, "[boot] MCPWM capture /dev/cap1 on GPIO %d\n", pin1);
    }
  }
#  endif

	usleep(10000);

	syslog(LOG_INFO, "PX4 PLATFORM CONFIGURE\n");
	px4_platform_configure();
	syslog(LOG_INFO, "PX4 PLATFORM CONFIGURE OK\n");

	led_off(LED_RED);
	led_on(LED_GREEN);          /* board init done, waiting for rcS */

	return OK;
}

#if defined(CONFIG_BOARD_LATE_INITIALIZE)
__EXPORT void board_late_initialize(void)
{
  (void)board_app_initialize(0);
}
#endif


