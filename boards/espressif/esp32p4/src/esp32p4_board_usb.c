/****************************************************************************
 * boards/espressif/esp32p4/nuttx-config/src/esp32p4_board_usb.c
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
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_USBDEV
#  include <nuttx/usb/usbdev.h>
#  include <nuttx/usb/usbdev_trace.h>
#endif

// /****************************************************************************
//  * Public Functions
//  ****************************************************************************/

#ifdef CONFIG_USBDEV
/****************************************************************************
 * Name: esp32p4_usbsuspend / esp_usbsuspend
 *
 * Description:
 *   Called when the USB device enters or leaves suspend mode.
 ****************************************************************************/

void esp32p4_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("USB suspend resume: %d\n", resume);
}

void esp_usbsuspend(struct usbdev_s *dev, bool resume)
  __attribute__((alias("esp32p4_usbsuspend")));
#endif
