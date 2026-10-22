/****************************************************************************
 * platforms/nuttx/src/px4/espressif/esp32p4/src/esp32p4_adc.c
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/param.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/mutex.h>
#include <nuttx/irq.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>


/* Espressif error types */
#include "esp_err.h"

#include "espressif/esp_gpio.h"
#include "esp32p4_adc.h"


/* Define Target and SOC evaluation macros for ESP32-P4 */
#ifndef CONFIG_IDF_TARGET_ESP32P4
#  define CONFIG_IDF_TARGET_ESP32P4 1
#endif

#ifndef _SOC_CAPS_TARGET_IS_ESP32P4
#  define _SOC_CAPS_TARGET_IS_ESP32P4 1
#endif

#ifndef SOC_IS
#  define _SOC_CAPS_EVAL(_name)               _SOC_CAPS_ ## _name
#  define SOC_IS(_target)                     _SOC_CAPS_EVAL(TARGET_IS_ ## _target)
#endif

/* Target evaluation constants */
#define _SOC_CAPS_TARGET_IS_ESP32             0
#define _SOC_CAPS_TARGET_IS_ESP32S2           0
#define _SOC_CAPS_TARGET_IS_ESP32S3           0
#define _SOC_CAPS_TARGET_IS_ESP32C3           0
#define _SOC_CAPS_TARGET_IS_ESP32C6           0
#define _SOC_CAPS_TARGET_IS_ESP32H2           0

#include "soc/soc_caps.h"
#include "soc/soc_caps_eval.h"
#include "sdkconfig.h"   /* if present under nuttx esp32p4 include */
/* Official ESP32-P4 Analog Converter HAL */
#include "hal/adc_types.h"
#include "hal/adc_oneshot_hal.h"
#include "soc/adc_channel.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_VAL_MASK            (0x0FFF)
#define ADC_FULL_SCALE_MV       (3100)   /* ~3.1V at 12dB attenuation */
#define ADC_FULL_COUNT          (4096)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct adc_chan_s
{
  uint32_t ref;                     /* Reference count */
  const uint8_t channel;            /* Channel number */
  const uint8_t pin;                /* GPIO pin number */
  const struct adc_callback_s *cb;  /* Upper driver callback */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  adc_bind(struct adc_dev_s *dev, const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind        = adc_bind,
  .ao_reset       = adc_reset,
  .ao_setup       = adc_setup,
  .ao_shutdown    = adc_shutdown,
  .ao_rxint       = adc_rxint,
  .ao_ioctl       = adc_ioctl,
};

/* ESP32-P4 ADC1 Channels (GPIO 16..23) */
static struct adc_chan_s g_adc1_chan0 = { .channel = 0, .pin = 16 };
static struct adc_chan_s g_adc1_chan1 = { .channel = 1, .pin = 17 };
static struct adc_chan_s g_adc1_chan2 = { .channel = 2, .pin = 18 };
static struct adc_chan_s g_adc1_chan3 = { .channel = 3, .pin = 19 };
static struct adc_chan_s g_adc1_chan4 = { .channel = 4, .pin = 20 };
static struct adc_chan_s g_adc1_chan5 = { .channel = 5, .pin = 21 };
static struct adc_chan_s g_adc1_chan6 = { .channel = 6, .pin = 22 };
static struct adc_chan_s g_adc1_chan7 = { .channel = 7, .pin = 23 };

static adc_oneshot_hal_ctx_t g_adc1_hal;
static bool g_hal_initialized = false;
static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

// static void adc_read_work(struct adc_dev_s *dev)
// {
//   int ret;
//   int raw_val = 0;
//   int32_t mv = 0;
//   struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

//   ret = nxmutex_lock(&g_lock);
//   if (ret < 0)
//     {
//       aerr("Failed to lock ADC: %d\n", ret);
//       return;
//     }

//   irqstate_t flags = enter_critical_section();
//   esp_err_t err = adc_oneshot_hal_convert(&g_adc1_hal, (adc_channel_t)priv->channel, &raw_val);
//   leave_critical_section(flags);

//   if (err == ESP_OK)
//     {
//       raw_val &= ADC_VAL_MASK;
//       mv = (int32_t)((raw_val * ADC_FULL_SCALE_MV) / ADC_FULL_COUNT);

//       if (priv->cb && priv->cb->au_receive)
//         {
//           priv->cb->au_receive(dev, priv->channel, mv);
//         }

//       ainfo("ADC1 ch %u (GPIO %u): raw=%d, %" PRIi32 " mV\n",
//             priv->channel, priv->pin, raw_val, mv);
//     }
//   else
//     {
//       aerr("ADC convert error: %d\n", err);
//     }

//   nxmutex_unlock(&g_lock);
// }


static void adc_read_work(struct adc_dev_s *dev)
{
  int ret;
  int raw_val = 0;
  int32_t mv = 0;
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      aerr("Failed to lock ADC: %d\n", ret);
      return;
    }

  irqstate_t flags = enter_critical_section();

  /* 1. Setup/select the channel to sample */
  adc_oneshot_hal_setup(&g_adc1_hal, (adc_channel_t)priv->channel);

  /* 2. Execute oneshot conversion */
  bool ok = adc_oneshot_hal_convert(&g_adc1_hal, &raw_val);

  leave_critical_section(flags);

  if (ok)
    {
      raw_val &= ADC_VAL_MASK;
      mv = (int32_t)((raw_val * ADC_FULL_SCALE_MV) / ADC_FULL_COUNT);

      if (priv->cb && priv->cb->au_receive)
        {
          priv->cb->au_receive(dev, priv->channel, mv);
        }

      ainfo("ADC1 ch %u (GPIO %u): raw=%d, %" PRIi32 " mV\n",
            priv->channel, priv->pin, raw_val, mv);
    }
  else
    {
      aerr("ADC convert error on ch %u\n", priv->channel);
    }

  nxmutex_unlock(&g_lock);
}


static int adc_bind(struct adc_dev_s *dev, const struct adc_callback_s *callback)
{
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;
  DEBUGASSERT(priv != NULL);

  priv->cb = callback;
  return OK;
}

static void adc_reset(struct adc_dev_s *dev)
{
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;
  ainfo("ADC reset ch: %u\n", priv->channel);
}

static int adc_setup(struct adc_dev_s *dev)
{
  int ret;
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->ref > 0)
    {
      priv->ref++;
      nxmutex_unlock(&g_lock);
      return OK;
    }

  /* Configure corresponding GPIO pin as analog input */
  esp_configgpio((int)priv->pin, INPUT);

  /* Initialize HAL layer once */
  if (!g_hal_initialized)
    {
      adc_oneshot_hal_cfg_t init_config =
      {
        .unit = ADC_UNIT_1,
        .work_mode = ADC_HAL_SINGLE_READ_MODE,
      };

      adc_oneshot_hal_init(&g_adc1_hal, &init_config);
      g_hal_initialized = true;
    }

  /* Configure channel attenuation (12dB) and resolution (12-bit) */
  adc_oneshot_hal_chan_cfg_t chan_cfg =
  {
    .atten    = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
  };

  adc_oneshot_hal_channel_config(&g_adc1_hal, &chan_cfg, (adc_channel_t)priv->channel);

  priv->ref++;
  nxmutex_unlock(&g_lock);

  return OK;
}

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  nxmutex_lock(&g_lock);
  if (priv->ref > 0)
    {
      priv->ref--;
    }
  nxmutex_unlock(&g_lock);
}

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
  /* Oneshot polled mode does not use RX interrupts */
}

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  int ret;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        adc_read_work(dev);
        ret = OK;
        break;

      case ANIOC_GET_NCHANNELS:
        ret = 1;
        break;

      default:
        aerr("ERROR: Unknown cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void esp32p4_adc_init(int adc_index, struct adc_dev_s *dev)
{
  if (!dev)
    {
      return;
    }

  dev->ad_ops = &g_adcops;

  switch (adc_index)
    {
      case 0:
        dev->ad_priv = &g_adc1_chan0;
        break;
      case 1:
        dev->ad_priv = &g_adc1_chan1;
        break;
      case 2:
        dev->ad_priv = &g_adc1_chan2;
        break;
      case 3:
        dev->ad_priv = &g_adc1_chan3;
        break;
      case 4:
        dev->ad_priv = &g_adc1_chan4;
        break;
      case 5:
        dev->ad_priv = &g_adc1_chan5;
        break;
      case 6:
        dev->ad_priv = &g_adc1_chan6;
        break;
      case 7:
        dev->ad_priv = &g_adc1_chan7;
        break;
      default:
        aerr("ERROR: Invalid ESP32-P4 ADC1 channel: %d\n", adc_index);
        dev->ad_priv = NULL;
        break;
    }
}
