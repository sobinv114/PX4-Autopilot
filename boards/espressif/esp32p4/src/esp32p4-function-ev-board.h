/****************************************************************************
 * boards/espressif/esp32p4/nuttx-config/src/esp32p4-function-ev-board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ESPRESSIF_ESP32P4_NUTTX_CONFIG_SRC_ESP32P4_FUNCTION_EV_BOARD_H
#define __BOARDS_ESPRESSIF_ESP32P4_NUTTX_CONFIG_SRC_ESP32P4_FUNCTION_EV_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32-P4 Function EV Board Hardware Pins *********************************/

/* BOOT Button (Active Low) */
#define BUTTON_BOOT             35


#ifndef GPIO_LED_RED
#  define GPIO_LED_RED  45
#endif

/* Status LEDs */
// #define GPIO_LED_RED            45
// #define GPIO_LED_GREEN          13
// #define GPIO_LED_BLUE           12

/* RMT Pins (if RMT driver is enabled) */
#define RMT_RXCHANNEL           4
#define RMT_TXCHANNEL           0
#define RMT_INPUT_PIN           2
#define RMT_OUTPUT_PIN          8

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif




/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_bringup / esp32p4_bringup
 *
 * Description:
 *   Perform architecture-specific initialization of board peripherals.
 *   Called during boot from board_late_initialize() or board_app_initialize().
 ****************************************************************************/

int esp_bringup(void);
int esp32p4_bringup(void);

/****************************************************************************
 * Name: esp_gpio_init / esp32p4_gpio_init
 *
 * Description:
 *   Configure and register the user GPIO pin driver (/dev/gpioX).
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int esp_gpio_init(void);
#endif

/****************************************************************************
 * Name: board_spiflash_init
 *
 * Description:
 *   Initialize external SPI flash and register MTD partitions.
 ****************************************************************************/

#if defined(CONFIG_ESPRESSIF_SPIFLASH) || defined(CONFIG_ESP32P4_SPIFLASH)
int board_spiflash_init(void);
#endif

/****************************************************************************
 * Name: board_i2c_init
 *
 * Description:
 *   Initialize I2C bus controllers and register /dev/i2cX devices.
 ****************************************************************************/

#if defined(CONFIG_I2C) || defined(CONFIG_ESPRESSIF_I2C0) || defined(CONFIG_ESPRESSIF_I2C1)
int board_i2c_init(void);
#endif

/****************************************************************************
 * Name: board_ledc_setup / esp32p4_pwm_setup
 *
 * Description:
 *   Initialize LEDC hardware and register PWM servo device (/dev/pwmX).
 ****************************************************************************/

#if defined(CONFIG_ESPRESSIF_LEDC) || defined(CONFIG_PWM)
int board_ledc_setup(void);
int esp32p4_pwm_setup(void);
#endif

/****************************************************************************
 * Name: board_adc_init / esp32p4_adc_init
 *
 * Description:
 *   Initialize ADC1 lower-half driver for voltage and current sensing.
 ****************************************************************************/

#if defined(CONFIG_ESPRESSIF_ADC) || defined(CONFIG_ADC)
int board_adc_init(void);
#endif

/****************************************************************************
 * Name: board_tim_init
 *
 * Description:
 *   Initialize hardware general purpose timers (/dev/timer0, /dev/timer1).
 ****************************************************************************/

#if defined(CONFIG_TIMER) || defined(CONFIG_ONESHOT)
int board_tim_init(void);
#endif

/****************************************************************************
 * Name: board_twai_setup
 *
 * Description:
 *   Initialize TWAI (CAN bus) interface and register /dev/canX.
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_TWAI
int board_twai_setup(int port);
#endif

/****************************************************************************
 * Name: board_emac_init
 *
 * Description:
 *   Initialize onboard Ethernet MAC (RMII / internal EMAC).
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_EMAC
int board_emac_init(void);
#endif

/****************************************************************************
 * Name: board_bmp180_initialize
 *
 * Description:
 *   Initialize and register the BMP180 barometric pressure sensor.
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP180
int board_bmp180_initialize(int devno, int busno);
#endif

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ESPRESSIF_ESP32P4_NUTTX_CONFIG_SRC_ESP32P4_FUNCTION_EV_BOARD_H */
