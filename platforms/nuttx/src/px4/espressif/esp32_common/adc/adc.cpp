/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <board_config.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/param.h>
#include <debug.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>
#include <px4_platform_common/log.h>
#include <nuttx/analog/adc.h>
#include <nuttx/spinlock.h>
#include <syslog.h>
#include <nuttx/semaphore.h>
#include <nuttx/irq.h>
#include <errno.h>


#include "esp_err.h"
// #include "esp_common/include/esp_err.h"
/* Real ESP32-P4 Analog Converter HAL headers */
#include "hal/adc_types.h"
#include "hal/adc_oneshot_hal.h"
#include "soc/adc_channel.h"

/* Max resolution is 12-bit (0..4095) */
#define ADC_VAL_MASK            (0x0FFF)

/* 12dB attenuation allows measuring up to ~3.1V */
#define ADC_ATTEN_DEFAULT       ADC_ATTEN_DB_12
#define ADC_BITWIDTH_DEFAULT    ADC_BITWIDTH_12

static adc_oneshot_hal_ctx_t g_adc1_hal_ctx;
static bool g_adc_initialized = false;

int px4_arch_adc_init(uint32_t base_address)
{
	if (g_adc_initialized) {
		return PX4_OK;
	}

	adc_oneshot_hal_cfg_t init_config = {
		.unit = ADC_UNIT_1,
		.work_mode = ADC_HAL_SINGLE_READ_MODE,
	};

	adc_oneshot_hal_init(&g_adc1_hal_ctx, &init_config);

#ifdef ADC_BATTERY_VOLTAGE_CHANNEL
	adc_oneshot_hal_chan_cfg_t volt_cfg = {
		.atten = ADC_ATTEN_DEFAULT,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	// adc_oneshot_hal_channel_config(&g_adc1_hal_ctx, &volt_cfg, ADC_BATTERY_VOLTAGE_CHANNEL);
	adc_oneshot_hal_channel_config(&g_adc1_hal_ctx, &volt_cfg, (adc_channel_t)ADC_BATTERY_VOLTAGE_CHANNEL);
#endif

#ifdef ADC_BATTERY_CURRENT_CHANNEL
	adc_oneshot_hal_chan_cfg_t curr_cfg = {
		.atten = ADC_ATTEN_DEFAULT,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	// adc_oneshot_hal_channel_config(&g_adc1_hal_ctx, &curr_cfg, ADC_BATTERY_CURRENT_CHANNEL);
	adc_oneshot_hal_channel_config(&g_adc1_hal_ctx, &curr_cfg, (adc_channel_t)ADC_BATTERY_CURRENT_CHANNEL);
#endif

	g_adc_initialized = true;
	PX4_INFO("ESP32-P4 ADC driver initialized successfully");

	return PX4_OK;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	g_adc_initialized = false;
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	if (!g_adc_initialized) {
		return UINT32_MAX;
	}

#ifdef ADC_BATTERY_CURRENT_CHANNEL
	if (channel == ADC_BATTERY_CURRENT_CHANNEL) {
		/* If current sensor is unpopulated or fixed, return dummy value or sample */
	}
#endif

	if (channel != ADC_BATTERY_VOLTAGE_CHANNEL
#ifdef ADC_BATTERY_CURRENT_CHANNEL
	    && channel != ADC_BATTERY_CURRENT_CHANNEL
#endif
	   ) {
		return UINT32_MAX;
	}

	// int raw = 0;
	// irqstate_t flags = enter_critical_section();

	// /* Read raw conversion using the P4 HAL */
	// esp_err_t ret = adc_oneshot_hal_convert(&g_adc1_hal_ctx, (adc_channel_t)channel, &raw);

	// leave_critical_section(flags);

	// if (ret != ESP_OK) {
	// 	return UINT32_MAX;
	// }

	// return (uint32_t)(raw & ADC_VAL_MASK);

	int raw = 0;
	irqstate_t flags = enter_critical_section();
	/* Channel already configured on g_adc1_hal_ctx during init */
	bool ok = adc_oneshot_hal_convert(&g_adc1_hal_ctx, &raw);
	leave_critical_section(flags);
	if (!ok) {
		return UINT32_MAX;
	}
	return (uint32_t)(raw & ADC_VAL_MASK);
}

float px4_arch_adc_reference_v(void)
{
	/* Full-scale reference voltage with ADC_ATTEN_DB_12 attenuation */
	return 3.1f;
}

uint32_t px4_arch_adc_temp_sensor_mask(void)
{
	return 0;
}

uint32_t px4_arch_adc_dn_fullcount(void)
{
	return 1 << 12; /* 4096 (12-bit ADC) */
}
