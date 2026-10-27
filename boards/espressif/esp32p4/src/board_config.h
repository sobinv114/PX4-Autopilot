/****************************************************************************
 *
 *   Copyright (c) 2013-2026 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * PX4 definitions for ESP32-P4 Function EV Board
 */

#pragma once


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nuttx/config.h"
#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#ifndef __ASSEMBLY__
#  include <nuttx/irq.h>
#  define px4_enter_critical_section()      enter_critical_section()
#  define px4_leave_critical_section(flags) leave_critical_section(flags)
#endif

#include "soc/adc_channel.h"
#include <px4_arch/micro_hal.h>


#define BOARD_HAVE_HW_SPLIT 1

/* ========================================================================= */
/* PX4IO Connection Configuration                                            */
/* ========================================================================= */

#define BOARD_USES_PX4IO_VERSION       2
/* Add static MFT so PX4IO is always “present” */
#define BOARD_HAS_STATIC_MANIFEST  1
#define PX4_MFT_HW_SUPPORTED_PX4_MFT_PX4IO  1
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS2"         /* Corresponding to UART2 */
#define PX4IO_SERIAL_TX_GPIO           GPIO_UART2_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_UART2_RX
#define PX4IO_SERIAL_BITRATE           1000000   /* Temporarily change from 1500000 */


/* ========================================================================= */
/* ESP32-P4 UART Pin Definitions                                            */
/* ========================================================================= */

#define GPIO_UART0_TX                   37
#define GPIO_UART0_RX                   38

#define GPIO_UART1_TX                   4
#define GPIO_UART1_RX                   5

#define GPIO_UART2_TX                   50
#define GPIO_UART2_RX                   49

#define GPIO_UART3_TX                   48
#define GPIO_UART3_RX                   49

#define GPIO_UART4_TX                   51
#define GPIO_UART4_RX                   52

#define GPIO_LPUART0_TX                 2
#define GPIO_LPUART0_RX                 3

/* ========================================================================= */
/* ESP32-P4 I2C Pin Definitions                                              */
/* ========================================================================= */

// #define BOARD_I2C_BUS_MAX_BUS_ITEMS    3
// #define I2C_BUS_MAX_BUS_ITEMS          BOARD_I2C_BUS_MAX_BUS_ITEMS

#define GPIO_I2C0_SDA                   22
#define GPIO_I2C0_SCL                   23

#define GPIO_I2C1_SCL                   32
#define GPIO_I2C1_SDA                   33

#define GPIO_LPI2C0_SDA                 7
#define GPIO_LPI2C0_SCL                 8

/* ========================================================================= */
/* ESP32-P4 SPI Pin Definitions                                              */
/* ========================================================================= */

/* SPI2 (Bus 2) */
#define GPIO_SPI2_CS                    28
#define GPIO_SPI2_MOSI                  29
#define GPIO_SPI2_CLK                   30
#define GPIO_SPI2_MISO                  31

/* SPI3 (Bus 3) */
#define GPIO_SPI3_CS                    20
#define GPIO_SPI3_MOSI                  21
#define GPIO_SPI3_CLK                   26
#define GPIO_SPI3_MISO                  27

#define BOARD_SPI_BUS_MAX_BUS_ITEMS     4

/* ========================================================================= */
/* Status LEDs                                                               */
/* Notice: GPIO 48 was used for LED in S3, but on P4 it is UART3 TX!         */
/* We use unassigned GPIOs (12, 13, 45) to avoid UART/SPI conflict.          */
/* ========================================================================= */

#define GPIO_LED_RED                    (GPIO_OUTPUT | 45)
#define GPIO_LED_GREEN                  (GPIO_OUTPUT | 13)
#define GPIO_LED_BLUE                   (GPIO_OUTPUT | 12)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_ARMED_LED                 LED_BLUE

/* ========================================================================= */
/* High Resolution Timer (HRT)                                              */
/* ========================================================================= */

#define HRT_TIMER                       0


/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
// #define PWMIN_TIMER          1          /* MUST be != HRT_TIMER */
// #define PWMIN_TIMER_CHANNEL  1
// #define GPIO_PWM_IN          (GPIO_INPUT | 9)

#define PWMIN_CAPTURE_DEVICE  "/dev/cap0"   /* or /dev/cap1 */


/* ========================================================================= */
/* ADC Channels for ESP32-P4                                                 */
/* ========================================================================= */

/* ADC1 Channels on ESP32-P4 (GPIO 16..23) */
#define ADC_BATTERY_VOLTAGE_CHANNEL     ADC1_GPIO16_CHANNEL    /* GPIO 16 = Channel 0 */
#define ADC_BATTERY_CURRENT_CHANNEL     ADC1_GPIO17_CHANNEL    /* GPIO 17 = Channel 1 */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))

#define ADC_V5_V_FULL_SCALE             (3.1f)

/* Heater Output */
#define GPIO_HEATER_OUTPUT              (GPIO_OUTPUT | 14)
#define HEATER_OUTPUT_EN(on_true)       px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* ========================================================================= */
/* PWM Servo / LEDC Timers                                                   */
/* ========================================================================= */

#define BOARD_NUM_IO_TIMERS             4
#define DIRECT_PWM_OUTPUT_CHANNELS      8

/* ========================================================================= */
/* Power Supply Monitoring Valid Flags                                       */
/* ========================================================================= */

#define BOARD_ADC_USB_CONNECTED         1
#define BOARD_ADC_BRICK_VALID           1
#define BOARD_ADC_USB_VALID             1

/* ========================================================================= */
/* Board Identity Dimensions                                                 */
/* ========================================================================= */

#define PX4_CPU_UUID_BYTE_LENGTH        12
#define PX4_CPU_UUID_WORD32_LENGTH      (PX4_CPU_UUID_BYTE_LENGTH / sizeof(uint32_t))
#define PX4_CPU_MFGUID_BYTE_LENGTH      PX4_CPU_UUID_BYTE_LENGTH

#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_CONSOLE_BUFFER_SIZE (1024*3)


/* Serial RGB LED (WS2812) */
/* Number of Neopixels on board (e.g. 1) */
#define BOARD_HAS_N_S_RGB_LED       CONFIG_WS2812_LED_COUNT
/* GPIO connected to Neopixel DIN */
#define BOARD_S_RGB_LED_GPIO        52
/* RMT channel to use (0 to 3) */
#define S_RGB_LED_RMT_CHANNEL       0
#define S_RGB_LED_BRIGHTNESS        3  /* 1..255, 255 = full */

// /* Tone alarm output */

// /* Tone alarm output (LEDC timer 2) */
// #define TONE_ALARM_LEDC_TIMER   2
// #define BUZZER_GPIO_NUM         12   /* same pin as CONFIG_ESPRESSIF_LEDC_CHANNEL2_PIN */

// #define GPIO_BUZZER_1           (GPIO_OUTPUT | BUZZER_GPIO_NUM)
// #define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1

/* ========================================================================= */
/* Tone Alarm / Buzzer Configuration                                        */
/* ========================================================================= */

#define TONE_ALARM_LEDC_TIMER       2    /* LEDC Timer 2 */
#define TONE_ALARM_LEDC_CHANNEL     2    /* LEDC Channel 2 */

#define BUZZER_GPIO_NUM             51   /* Matches CONFIG_ESPRESSIF_LEDC_CHANNEL2_PIN */

/* Idle state configuration: GPIO Output, Pulled Low */
#define GPIO_TONE_ALARM_IDLE        (GPIO_OUTPUT | GPIO_PULLDOWN | BUZZER_GPIO_NUM)


/* DShot Motor Output Pins */
#define GPIO_DSHOT_CH0    15
#define GPIO_DSHOT_CH1    16
#define GPIO_DSHOT_CH2    17
#define GPIO_DSHOT_CH3    18



/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
/* Enable safety switch and button support linked through PX4IO / FMU */
#define GPIO_nSAFETY_SWITCH_LED_OUT    (GPIO_OUTPUT | 10)    /* Example pin for safety LED */
#define GPIO_SAFETY_SWITCH_IN          (GPIO_INPUT | 9)      /* Example pin for safety button */
#define GPIO_LED_SAFETY                GPIO_nSAFETY_SWITCH_LED_OUT
#define GPIO_BTN_SAFETY                GPIO_SAFETY_SWITCH_IN




/* Power switch controls ******************************************************/


/* SDMMC initialization function */
#ifdef CONFIG_MMCSD
int esp32p4_sdio_initialize(void);
#endif



__BEGIN_DECLS

#ifndef __ASSEMBLY__


extern void esp32p4_spiinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>


/* Undefine the fallback macros from board_common.h */
#ifdef board_determine_hw_info
#  undef board_determine_hw_info
#endif

#ifdef board_get_hw_revision
#  undef board_get_hw_revision
#endif

#ifdef board_get_hw_version
#  undef board_get_hw_version
#endif

#ifdef board_get_hw_type_name
#  undef board_get_hw_type_name
#endif

/* Provide board-specific function declarations */
__BEGIN_DECLS
__EXPORT int board_determine_hw_info(void);
__EXPORT int board_get_hw_revision(void);
__EXPORT int board_get_hw_version(void);
__EXPORT const char *board_get_hw_type_name(void);
__END_DECLS


#endif /* __ASSEMBLY__ */

__END_DECLS
