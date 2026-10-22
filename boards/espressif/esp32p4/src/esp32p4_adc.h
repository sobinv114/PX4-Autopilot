/****************************************************************************
 * platforms/nuttx/src/px4/espressif/esp32p4/include/esp32p4_adc.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __PLATFORMS_NUTTX_SRC_PX4_ESPRESSIF_ESP32P4_INCLUDE_ESP32P4_ADC_H
#define __PLATFORMS_NUTTX_SRC_PX4_ESPRESSIF_ESP32P4_INCLUDE_ESP32P4_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32-P4 ADC1 Channels mapped to GPIO 16..23 */
#define ESP32P4_ADC1               1
#define ESP32P4_ADC1_CHANNEL0      0  /* GPIO 16 */
#define ESP32P4_ADC1_CHANNEL1      1  /* GPIO 17 */
#define ESP32P4_ADC1_CHANNEL2      2  /* GPIO 18 */
#define ESP32P4_ADC1_CHANNEL3      3  /* GPIO 19 */
#define ESP32P4_ADC1_CHANNEL4      4  /* GPIO 20 */
#define ESP32P4_ADC1_CHANNEL5      5  /* GPIO 21 */
#define ESP32P4_ADC1_CHANNEL6      6  /* GPIO 22 */
#define ESP32P4_ADC1_CHANNEL7      7  /* GPIO 23 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: esp32p4_adc_init
 *
 * Description:
 *   Initialize the ADC lower-half device for the given ADC channel.
 *
 * Input Parameters:
 *   adc_index - ADC channel number (0..7)
 *   dev       - Pointer to device structure used by the upper-half driver
 ****************************************************************************/

void esp32p4_adc_init(int adc_index, struct adc_dev_s *dev);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __PLATFORMS_NUTTX_SRC_PX4_ESPRESSIF_ESP32P4_INCLUDE_ESP32P4_ADC_H */
