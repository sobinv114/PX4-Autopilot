/****************************************************************************
 *
 *   Copyright (c) 2019-2026 PX4 Development Team. All rights reserved.
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
#pragma once

#include <board_config.h>
#include <stdint.h>
#include <stdbool.h>

#if !defined(HW_REV_VER_ADC_BASE)
#  define HW_REV_VER_ADC_BASE 0
#endif

#if !defined(SYSTEM_ADC_BASE)
#  define SYSTEM_ADC_BASE 0
#endif

/* ESP32-P4 has 55 GPIOs (0 to 54) */
#define GPIO_PIN_COUNT                  55

typedef enum {
    ADC_0db = 0,
    ADC_2_5db,
    ADC_6db,
    ADC_11db,
    ADC_12db = ADC_11db
} adc_attenuation_t;

typedef struct {
    int8_t adc_channel;   /*!< ADC1 Channel number (-1 if not an ADC pin) */
    int8_t touch_channel; /*!< Touch Channel number (-1 if not a Touch pin) */
} esp32p4_gpioMux_t;

/* ADC1 Channels on ESP32-P4 are mapped to GPIO 16..23 */
#define digitalPinToAnalogChannel(pin) (((pin) >= 16 && (pin) <= 23) ? ((pin) - 16) : -1)
#define digitalPinToTouchChannel(pin)  (-1)

bool adcAttachPin(uint8_t pin);
uint16_t analogReadPin(uint8_t pin);
uint16_t analogRead(uint8_t channel);

#include <px4_platform/adc.h>
