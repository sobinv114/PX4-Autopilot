/****************************************************************************
 *
 *   Copyright (c) 2021-2026 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>

#include <arch/board/board.h>
#include <arch/chip/chip.h>
#include <arch/chip/irq.h>
#include <px4_arch/micro_hal.h>
#include <errno.h>
#include <stdio.h>
#include <syslog.h>

#include "esp_gpio.h"
// #include "espressif/esp_gpio.h"
#include "soc/soc_caps.h"



#ifndef ESP32P4_NGPIOS
#  define ESP32P4_NGPIOS SOC_GPIO_PIN_COUNT /* 55 (GPIO 0-54) */
#endif

int px4_esp32p4_configgpio(uint32_t pinset)
{
	uint32_t pin = pinset & GPIO_NUM_MASK;

	if (pin >= ESP32P4_NGPIOS) {
		return -EINVAL;
	}

	gpio_pinattr_t attr = (gpio_pinattr_t)((pinset >> GPIO_SET_SHIFT) & 0xffff);
	return esp_configgpio((int)pin, attr);
}

int px4_esp32p4_unconfiggpio(uint32_t pinset)
{
	return px4_esp32p4_configgpio((pinset & GPIO_NUM_MASK) | GPIO_INPUT | GPIO_OPEN_DRAIN);
}

/****************************************************************************
 * Name: esp32p4_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 ****************************************************************************/
int esp32p4_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                         bool event, xcpt_t func, void *arg)
{
	uint32_t pin = pinset & GPIO_NUM_MASK;

	if (pin >= ESP32P4_NGPIOS) {
		return -EINVAL;
	}

	int irq = ESP_PIN2IRQ(pin);

	if (event == true) {
		int ret = irq_attach(irq, func, arg);
		if (ret < 0) {
			syslog(LOG_ERR, "ERROR: irq_attach() failed for pin %" PRIu32 ": %d\n", pin, ret);
			return ret;
		}

		// /* Determine trigger type */
		// gpio_pinattr_t intr_type = DISABLED;
		// if (risingedge && fallingedge) {
		// 	intr_type = CHANGE;
		// } else if (risingedge) {
		// 	intr_type = RISING;
		// } else if (fallingedge) {
		// 	intr_type = FALLING;
		// }

	        /* Interrupt type bits (same as NuttX esp_gpio.h: shift 12) */
		gpio_pinattr_t intr_type = 0;           /* DISABLED */
		if (risingedge && fallingedge) {
			intr_type = (3 << 12);         /* CHANGE */
		} else if (risingedge) {
			intr_type = (1 << 12);         /* RISING */
		} else if (fallingedge) {
			intr_type = (2 << 12);         /* FALLING */
		}

		/* Configure the pin attribute with interrupt mode and enable */
		gpio_pinattr_t attr = INPUT | intr_type;
		esp_configgpio((int)pin, attr);
		esp_gpioirqenable((int)pin);
	} else {
		esp_gpioirqdisable((int)pin);
	}

	return OK;
}
