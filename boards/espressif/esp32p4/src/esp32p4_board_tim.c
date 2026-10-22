/****************************************************************************
 * boards/espressif/esp32p4/nuttx-config/src/esp32p4_board_tim.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <debug.h>
#include <syslog.h>
#include <errno.h>


#ifdef CONFIG_ONESHOT
#include <nuttx/timers/oneshot.h>
#ifdef CONFIG_CPULOAD_ONESHOT
#include <nuttx/clock.h>
#endif
#endif

#include "esp32p4_board_tim.h"

/* ESP32-P4 / RISC-V Timer Subsystem */
#include "espressif/esp_gptimer.h"

#ifdef CONFIG_ONESHOT
#include "espressif/esp_oneshot.h"
#endif

#ifndef ONESHOT_TIMER_NUM
#  define ONESHOT_TIMER_NUM      0     /* hardware timer index used by oneshot */
#endif

#ifndef ONESHOT_RESOLUTION_US
#  define ONESHOT_RESOLUTION_US  1     /* 1 us tick */
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: config_oneshot_timer
 *
 * Description:
 *   Configure the oneshot timer driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ONESHOT
static int config_oneshot_timer(int timer, uint16_t resolution)
{
  int ret = OK;
  struct oneshot_lowerhalf_s *os_lower = NULL;

  os_lower = oneshot_initialize(timer, resolution);
  if (os_lower == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize oneshot timer.\n");
      return -EBUSY;
    }

#if defined(CONFIG_CPULOAD_ONESHOT)
  /* Configure the oneshot timer to support CPU load measurement */

  nxsched_oneshot_extclk(os_lower);
#else
  ret = oneshot_register("/dev/oneshot", os_lower);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register oneshot device: %d\n", ret);
    }
#endif /* CONFIG_CPULOAD_ONESHOT */

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_tim_init
 *
 * Description:
 *   Configure the ESP32-P4 timer driver(s) and register /dev/timerX devices.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 ****************************************************************************/


int board_tim_init(void)
{
  int ret = OK;

#if defined(CONFIG_TIMER) && !defined(CONFIG_ONESHOT)

#  if defined(CONFIG_ESPRESSIF_LEDC_TIMER0)
  /* Initialize hardware Timer 0 -> registers /dev/timer0 */

  ret = esp_timer_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize Timer 0: %d\n", ret);
    }
#  endif

#  if defined(CONFIG_ESPRESSIF_LEDC_TIMER1)
  ret = esp_timer_initialize(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize Timer 1: %d\n", ret);
    }
#  endif

#  if defined(CONFIG_ESPRESSIF_LEDC_TIMER2)
  ret = esp_timer_initialize(2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize Timer 2: %d\n", ret);
    }
#  endif

#  if defined(CONFIG_ESPRESSIF_LEDC_TIMER3)
  ret = esp_timer_initialize(3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize Timer 3: %d\n", ret);
    }
#  endif

#endif /* CONFIG_TIMER && !CONFIG_ONESHOT */

#if defined(CONFIG_ONESHOT)
  /* Initialize oneshot timer driver -> registers /dev/oneshot */

  ret = config_oneshot_timer(ONESHOT_TIMER_NUM, ONESHOT_RESOLUTION_US);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize Oneshot Timer: %d\n", ret);
    }
#endif /* CONFIG_ONESHOT */

  return ret;
}


// int board_tim_init(void)
// {
//   int ret = OK;

// #ifdef (CONFIG_TIMER)  && defined(CONFIG_ESPRESSIF_LEDC_TIMER0) && !defined(CONFIG_ONESHOT)
//   /* Initialize hardware Timer 0 -> registers /dev/timer0 */

//   ret = esp_timer_initialize(0);
//   if (ret < 0)
//     {
//       syslog(LOG_ERR, "Failed to initialize Timer 0: %d\n", ret);
//     }

// #ifdef (CONFIG_TIMER)  && defined(CONFIG_ESPRESSIF_LEDC_TIMER1)

//   /* Initialize hardware Timer 1 -> registers /dev/timer1 */

//   ret = esp_timer_initialize(1);
//   if (ret < 0)
//     {
//       syslog(LOG_ERR, "Failed to initialize Timer 0: %d\n", ret);
//     }

// #ifdef (CONFIG_TIMER)  && defined(CONFIG_ESPRESSIF_LEDC_TIMER2)

//   /* Initialize hardware Timer 2 -> registers /dev/timer2 */

//   ret = esp_timer_initialize(2);
//   if (ret < 0)
//     {
//       syslog(LOG_ERR, "Failed to initialize Timer 2: %d\n", ret);
//     }


// #ifdef (CONFIG_TIMER)  && defined(CONFIG_ESPRESSIF_LEDC_TIMER3)
//   /* Initialize hardware Timer 3 -> registers /dev/timer3 */

//   ret = esp_timer_initialize(3);
//   if (ret < 0)
//     {
//       syslog(LOG_ERR, "Failed to initialize Timer 3: %d\n", ret);
//     }


// #ifdef (CONFIG_TIMER)  && defined(CONFIG_ESPRESSIF_LEDC_TIMER0) && defined(CONFIG_ONESHOT)
//   /* Initialize oneshot timer driver -> registers /dev/oneshot */

//   ret = esp_oneshot_initialize();
//   if (ret < 0)
//     {
//       syslog(LOG_ERR, "Failed to initialize Oneshot Timer: %d\n", ret);
//     }
// #endif /* CONFIG_ONESHOT */

//   return ret;
// }
