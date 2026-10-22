/****************************************************************************
 * boards/espressif/esp32p4/nuttx-config/include/board.h
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT      2  /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT      1  /* Amount of GPIO Input w/ Interruption pins */

/* ESP32-P4 Function EV Board Buttons */

#define BUTTON_BOOT         35 /* Boot button GPIO pin on ESP32-P4 */

/* LED pattern definitions for CONFIG_ARCH_LEDS */

#define LED_STARTED         0  /* System started */
#define LED_HEAPALLOCATE    1  /* Heap allocated */
#define LED_IRQSENABLED     2  /* Interrupts enabled */
#define LED_STACKCREATED    3  /* Initialization thread created */
#define LED_INIRQ           4  /* In interrupt handler */
#define LED_SIGNAL          5  /* In signal handler */
#define LED_ASSERTION       6  /* Assertion failure */
#define LED_PANIC           7  /* Fatal crash / panic */

/****************************************************************************
 * Public Function Prototypes
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
 * Name: board_bringup
 *
 * Description:
 *   Perform architecture-specific initialization of board peripherals.
 ****************************************************************************/

int board_bringup(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_BOARD_BOARD_H */
