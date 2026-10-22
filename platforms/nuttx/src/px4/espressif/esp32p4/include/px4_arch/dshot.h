/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_config.h>
#include <stdint.h>
#include <stdbool.h>

__BEGIN_DECLS

int up_dshot_init(uint32_t channel_mask, uint32_t bdshot_channel_mask, unsigned dshot_pwm_freq, bool edt_enable);
void dshot_motor_data_set(uint8_t channel, uint16_t throttle, bool telemetry);
void up_dshot_trigger(void);
int up_dshot_arm(bool armed);

uint16_t up_bdshot_get_ready_mask(void);
int up_bdshot_num_errors(uint8_t channel);
int up_bdshot_get_erpm(uint8_t channel, int *erpm);
int up_bdshot_get_extended_telemetry(uint8_t channel, int type, uint8_t *value);
int up_bdshot_channel_online(uint8_t channel);
int up_bdshot_channel_capture_supported(uint8_t channel);
void up_bdshot_status(void);

__END_DECLS
