/****************************************************************************
 *
 * Copyright (C) 2024-2026 PX4 Development Team. All rights reserved.
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
 * @file dshot.c
 * Architecture implementation for DShot / Bidirectional DShot on ESP32-P4
 * using register-level RMT access and GPIO matrix routing.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/log.h>
#include <board_config.h>
#include <drivers/drv_dshot.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/dshot.h>
#include <lib/perf/perf_counter.h>

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* ========================================================================= */
/* ESP32-P4 Hardware RMT Register Definitions (Direct MMIO)                  */
/* ========================================================================= */

/* NuttX GPIO flag compatibility across architectures */
#ifndef GPIO_OPEN_DRAIN
#  ifdef GPIO_OPENDRAIN
#    define GPIO_OPEN_DRAIN GPIO_OPENDRAIN
#  else
#    define GPIO_OPEN_DRAIN (1 << 8)
#  endif
#endif

#define ESP32P4_RMT_BASE            0x500A4000u
#define ESP32P4_RMT_MEM_BASE        0x500A4400u

#define RMT_REG(offset)             (*(volatile uint32_t *)(ESP32P4_RMT_BASE + (offset)))
#define RMT_MEM(ch, idx)            (*(volatile uint32_t *)(ESP32P4_RMT_MEM_BASE + ((ch) * 192u) + ((idx) * 4u)))

/* Register Offsets */
#define RMT_CH_TX_CONF0_REG(ch)     ((ch) * 0x08u + 0x00u)
#define RMT_CH_RX_CONF0_REG(ch)     ((ch) * 0x08u + 0x20u)
#define RMT_SYS_CONF_REG            0x00C0u

/* RMT Register Bitmasks */
#define RMT_TX_START_BIT            (1u << 0)
#define RMT_TX_RESET_CNT_BIT        (1u << 1)
#define RMT_TX_IDLE_OUT_EN_BIT      (1u << 3)
#define RMT_TX_IDLE_OUT_LVL_BIT     (1u << 4)

#define RMT_RX_EN_BIT               (1u << 0)
#define RMT_RX_RESET_CNT_BIT        (1u << 1)
#define RMT_RX_FILTER_EN_BIT        (1u << 4)

/* 32-bit RMT Pulse Symbol Definition */
typedef union {
	struct {
		uint16_t duration0 : 15;
		uint16_t level0    :  1;
		uint16_t duration1 : 15;
		uint16_t level1    :  1;
	};
	uint32_t val;
} rmt_symbol_word_t;

/* ========================================================================= */
/* Protocol Constants & Lookup Tables                                        */
/* ========================================================================= */

#define DSHOT_FRAME_SIZE            16u
#define DSHOT_THROTTLE_POSITION     5u
#define DSHOT_TELEMETRY_POSITION    4u
#define NIBBLES_SIZE                4u
#define DSHOT_NUMBER_OF_NIBBLES     3u
#define MAX_DSHOT_CHANNELS          4u

#define BDSHOT_OFFLINE_COUNT        200
#define RMT_RX_MEM_WORDS            48u

static const uint32_t gcr_decode[32] = {
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x9, 0xA, 0xB, 0x0, 0xD, 0xE, 0xF,
	0x0, 0x0, 0x2, 0x3, 0x0, 0x5, 0x6, 0x7,
	0x0, 0x0, 0x8, 0x1, 0x0, 0x4, 0xC, 0x0
};

typedef struct {
	int32_t erpm;
	bool ready;
	float rate_hz;
	uint64_t last_timestamp;
} erpm_data_t;

typedef struct {
	uint8_t value;
	bool ready;
	float rate_hz;
	uint64_t last_timestamp;
} edt_data_t;

typedef struct {
	bool enabled;
	bool bdshot;
	int gpio;
	uint8_t tx_chan;
	uint8_t rx_chan;

	rmt_symbol_word_t tx_symbols[DSHOT_FRAME_SIZE + 1];

	bool online;
	uint32_t read_ok;
	uint32_t read_fail_crc;
	uint32_t no_response_cnt;
	uint16_t consecutive_successes;
	uint16_t consecutive_failures;
} dshot_channel_t;

static dshot_channel_t _channels[MAX_DSHOT_CHANNELS] = {0};
static uint32_t _channel_mask = 0;
static uint32_t _bdshot_channel_mask = 0;
static volatile uint16_t _bdshot_ready_mask = 0;
static unsigned _dshot_frequency = 600000;
static bool _edt_enabled = false;
static bool _dshot_armed = false;

static uint16_t _t0h_ticks = 0;
static uint16_t _t0l_ticks = 0;
static uint16_t _t1h_ticks = 0;
static uint16_t _t1l_ticks = 0;
static hrt_abstime _bdshot_capture_delay = 0;

static uint32_t _base_interval_x8[MAX_DSHOT_CHANNELS] = { 168, 168, 168, 168 };

static volatile erpm_data_t _erpms[MAX_DSHOT_CHANNELS] = {0};
static volatile edt_data_t  _edt_temp[MAX_DSHOT_CHANNELS] = {0};
static volatile edt_data_t  _edt_volt[MAX_DSHOT_CHANNELS] = {0};
static volatile edt_data_t  _edt_curr[MAX_DSHOT_CHANNELS] = {0};
static volatile edt_data_t  _edt_state[MAX_DSHOT_CHANNELS] = {0};

static struct hrt_call _cc_calls;
static volatile bool _cycle_complete = true;

static perf_counter_t _burst_perf = NULL;
static perf_counter_t _capture_perf = NULL;

#ifndef GPIO_DSHOT_CH0
#  define GPIO_DSHOT_CH0 15
#endif
#ifndef GPIO_DSHOT_CH1
#  define GPIO_DSHOT_CH1 16
#endif
#ifndef GPIO_DSHOT_CH2
#  define GPIO_DSHOT_CH2 17
#endif
#ifndef GPIO_DSHOT_CH3
#  define GPIO_DSHOT_CH3 18
#endif

static const int g_default_gpios[MAX_DSHOT_CHANNELS] = {
	GPIO_DSHOT_CH0,
	GPIO_DSHOT_CH1,
	GPIO_DSHOT_CH2,
	GPIO_DSHOT_CH3
};

static void capture_complete_callback(void *arg);
static void process_bdshot_response(uint8_t channel);
static float calculate_rate_hz(uint64_t last_timestamp, float last_rate_hz, uint64_t timestamp);
static void decode_dshot_telemetry(uint32_t payload, struct BDShotTelemetry *packet);

/* ------------------------------------------------------------------------- */
/* Hardware RMT Control Functions                                            */
/* ------------------------------------------------------------------------- */

static void hw_rmt_init_channel_tx(uint8_t chan, bool inverted)
{
	/* Divisor = 1 (80MHz tick = 12.5ns), Idle output enabled */
	uint32_t conf = (1u << 8) | RMT_TX_IDLE_OUT_EN_BIT;

	if (inverted) {
		conf |= RMT_TX_IDLE_OUT_LVL_BIT;
	}

	RMT_REG(RMT_CH_TX_CONF0_REG(chan)) = conf;
}

static void hw_rmt_init_channel_rx(uint8_t chan)
{
	/* Divisor = 1, Filter enabled (threshold = 8 ticks / 100ns) */
	uint32_t conf = (1u << 8) | RMT_RX_FILTER_EN_BIT | (8u << 16);
	RMT_REG(RMT_CH_RX_CONF0_REG(chan)) = conf;
}

static inline void hw_rmt_start_tx(uint8_t chan)
{
	RMT_REG(RMT_CH_TX_CONF0_REG(chan)) |= (RMT_TX_RESET_CNT_BIT | RMT_TX_START_BIT);
}

static inline void hw_rmt_start_rx(uint8_t chan)
{
	RMT_REG(RMT_CH_RX_CONF0_REG(chan)) |= (RMT_RX_RESET_CNT_BIT | RMT_RX_EN_BIT);
}

static inline void hw_rmt_stop_rx(uint8_t chan)
{
	RMT_REG(RMT_CH_RX_CONF0_REG(chan)) &= ~RMT_RX_EN_BIT;
}

static void hw_rmt_write_symbols(uint8_t chan, const rmt_symbol_word_t *symbols, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		RMT_MEM(chan, i) = symbols[i].val;
	}
}

/* ------------------------------------------------------------------------- */
/* PX4 DShot Architecture API                                                */
/* ------------------------------------------------------------------------- */

int up_dshot_init(uint32_t channel_mask, uint32_t bdshot_channel_mask, unsigned dshot_pwm_freq, bool edt_enable)
{
	_dshot_frequency = dshot_pwm_freq;
	_channel_mask = channel_mask;
	_bdshot_channel_mask = bdshot_channel_mask;
	_edt_enabled = edt_enable;
	_cycle_complete = true;
	_dshot_armed = false;
	_bdshot_ready_mask = 0;

	/* 80 MHz APB RMT source clock */
	const uint32_t rmt_clk_hz = 80000000;
	uint32_t total_ticks = rmt_clk_hz / _dshot_frequency;

	_t0h_ticks = (total_ticks * 375) / 1000;
	_t0l_ticks = total_ticks - _t0h_ticks;

	_t1h_ticks = (total_ticks * 750) / 1000;
	_t1l_ticks = total_ticks - _t1h_ticks;

	hrt_abstime frame_us = (16 * 1000000) / _dshot_frequency;
	_bdshot_capture_delay = frame_us + 30 + ((21 * 4 * 1000000) / (5 * _dshot_frequency)) + 25;

	if (_bdshot_channel_mask) {
		_burst_perf   = perf_alloc(PC_ELAPSED, "dshot_esp32_burst");
		_capture_perf = perf_alloc(PC_ELAPSED, "dshot_esp32_capture");
	}

	int initialized_mask = 0;

	for (uint8_t i = 0; i < MAX_DSHOT_CHANNELS; i++) {
		if (_channel_mask & (1 << i)) {
			dshot_channel_t *ch = &_channels[i];
			ch->enabled  = true;
			ch->gpio     = g_default_gpios[i];
			ch->tx_chan  = i;
			ch->rx_chan  = i;
			ch->bdshot   = (_bdshot_channel_mask & (1 << i)) != 0;

			if (ch->bdshot) {
				px4_arch_configgpio(GPIO_OUTPUT | GPIO_OPEN_DRAIN | GPIO_PULLUP | ch->gpio);
				hw_rmt_init_channel_rx(ch->rx_chan);
				hw_rmt_init_channel_tx(ch->tx_chan, true);
				_bdshot_ready_mask |= (1 << i);

			} else {
				px4_arch_configgpio(GPIO_OUTPUT | ch->gpio);
				hw_rmt_init_channel_tx(ch->tx_chan, false);
			}

			ch->tx_symbols[DSHOT_FRAME_SIZE].duration0 = 0;
			ch->tx_symbols[DSHOT_FRAME_SIZE].level0    = ch->bdshot ? 1 : 0;
			ch->tx_symbols[DSHOT_FRAME_SIZE].duration1 = 0;
			ch->tx_symbols[DSHOT_FRAME_SIZE].level1    = ch->bdshot ? 1 : 0;

			initialized_mask |= (1 << i);
		}
	}

	PX4_INFO("ESP32-P4 DShot initialized: freq=%uHz, mask=0x%02x", _dshot_frequency, (unsigned)initialized_mask);
	return initialized_mask;
}

void dshot_motor_data_set(uint8_t channel, uint16_t throttle, bool telemetry)
{
	if (channel >= MAX_DSHOT_CHANNELS || !_channels[channel].enabled) {
		return;
	}

	dshot_channel_t *ch = &_channels[channel];

	uint16_t packet = 0;
	packet |= (throttle << DSHOT_THROTTLE_POSITION);
	packet |= ((telemetry ? 1u : 0u) << DSHOT_TELEMETRY_POSITION);

	uint16_t csum = 0;
	uint16_t csum_data = ch->bdshot ? ~packet : packet;
	csum_data >>= NIBBLES_SIZE;

	for (uint8_t i = 0; i < DSHOT_NUMBER_OF_NIBBLES; i++) {
		csum ^= (csum_data & 0x0F);
		csum_data >>= NIBBLES_SIZE;
	}

	packet |= (csum & 0x0F);

	bool inverted = ch->bdshot;

	for (uint8_t bit = 0; bit < DSHOT_FRAME_SIZE; bit++) {
		bool bit_val = (packet & (1 << (15 - bit))) != 0;

		if (bit_val) {
			ch->tx_symbols[bit].duration0 = _t1h_ticks;
			ch->tx_symbols[bit].level0    = inverted ? 0 : 1;
			ch->tx_symbols[bit].duration1 = _t1l_ticks;
			ch->tx_symbols[bit].level1    = inverted ? 1 : 0;

		} else {
			ch->tx_symbols[bit].duration0 = _t0h_ticks;
			ch->tx_symbols[bit].level0    = inverted ? 0 : 1;
			ch->tx_symbols[bit].duration1 = _t0l_ticks;
			ch->tx_symbols[bit].level1    = inverted ? 1 : 0;
		}
	}

	hw_rmt_write_symbols(ch->tx_chan, ch->tx_symbols, DSHOT_FRAME_SIZE + 1);
}

void up_dshot_trigger(void)
{
	if (!_dshot_armed) {
		return;
	}

	if (!_cycle_complete) {
		PX4_WARN("DShot cycle incomplete!");
		return;
	}

	_cycle_complete = false;
	perf_begin(_burst_perf);

	/* 1. Arm RX channels for Bidirectional DShot */
	for (uint8_t i = 0; i < MAX_DSHOT_CHANNELS; i++) {
		if (_channels[i].enabled && _channels[i].bdshot) {
			hw_rmt_start_rx(_channels[i].rx_chan);
		}
	}

	/* 2. Fire TX channels simultaneously */
	for (uint8_t i = 0; i < MAX_DSHOT_CHANNELS; i++) {
		if (_channels[i].enabled) {
			hw_rmt_start_tx(_channels[i].tx_chan);
		}
	}

	perf_end(_burst_perf);

	if (_bdshot_channel_mask) {
		hrt_call_after(&_cc_calls, _bdshot_capture_delay, capture_complete_callback, NULL);

	} else {
		_cycle_complete = true;
	}
}

static void capture_complete_callback(void *arg)
{
	(void)arg;
	perf_begin(_capture_perf);

	for (uint8_t i = 0; i < MAX_DSHOT_CHANNELS; i++) {
		if (_channels[i].enabled && _channels[i].bdshot) {
			hw_rmt_stop_rx(_channels[i].rx_chan);
			process_bdshot_response(i);
		}
	}

	perf_end(_capture_perf);
	_cycle_complete = true;
}

static void process_bdshot_response(uint8_t channel)
{
	dshot_channel_t *ch = &_channels[channel];

	uint32_t intervals[RMT_RX_MEM_WORDS * 2];
	unsigned interval_count = 0;

	for (size_t i = 0; i < RMT_RX_MEM_WORDS; i++) {
		uint32_t word = RMT_MEM(ch->rx_chan, i);
		uint16_t d0 = word & 0x7FFF;
		uint16_t d1 = (word >> 16) & 0x7FFF;

		if (d0 == 0) break;
		if (d0 >= 10 && d0 <= 200) intervals[interval_count++] = d0;

		if (d1 == 0) break;
		if (d1 >= 10 && d1 <= 200) intervals[interval_count++] = d1;
	}

	if (interval_count < 18) {
		ch->no_response_cnt++;

		if (ch->consecutive_failures++ > BDSHOT_OFFLINE_COUNT) {
			ch->consecutive_failures = BDSHOT_OFFLINE_COUNT;
			ch->consecutive_successes = 0;
			ch->online = false;
		}

		_bdshot_ready_mask |= (1u << channel);
		return;
	}

	uint32_t min_interval = intervals[0];
	for (unsigned i = 1; i < interval_count; i++) {
		if (intervals[i] < min_interval) {
			min_interval = intervals[i];
		}
	}

	_base_interval_x8[channel] = _base_interval_x8[channel] - (_base_interval_x8[channel] >> 3) + min_interval;
	uint32_t base_x8 = _base_interval_x8[channel];

	uint32_t value = 0;
	uint32_t high = 1;
	unsigned shifted = 0;

	for (unsigned i = 0; i < interval_count; i++) {
		uint32_t bits = (intervals[i] * 8 + base_x8 / 2) / base_x8;

		if (bits < 1) bits = 1;
		else if (bits > 4) bits = 4;

		for (unsigned b = 0; b < bits; ++b) {
			value = (value << 1) | high;
			shifted++;
		}
		high = !high;
	}

	if (shifted < 18 || shifted > 24) {
		ch->read_fail_crc++;
		_bdshot_ready_mask |= (1u << channel);
		return;
	}

	if (shifted < 21) {
		value <<= (21 - shifted);
	} else if (shifted > 21) {
		value >>= (shifted - 21);
	}

	value = (value ^ (value >> 1));

	uint32_t payload = gcr_decode[value & 0x1f];
	payload |= gcr_decode[(value >> 5) & 0x1f] << 4;
	payload |= gcr_decode[(value >> 10) & 0x1f] << 8;
	payload |= gcr_decode[(value >> 15) & 0x1f] << 12;

	uint32_t checksum = payload;
	checksum = checksum ^ (checksum >> 8);
	checksum = checksum ^ (checksum >> NIBBLES_SIZE);

	if ((checksum & 0xF) != 0xF) {
		ch->read_fail_crc++;

		if (ch->consecutive_failures++ > BDSHOT_OFFLINE_COUNT) {
			ch->consecutive_failures = BDSHOT_OFFLINE_COUNT;
			ch->consecutive_successes = 0;
			ch->online = false;
		}

		_bdshot_ready_mask |= (1u << channel);
		return;
	}

	ch->read_ok++;

	if (ch->consecutive_successes++ > BDSHOT_OFFLINE_COUNT) {
		ch->consecutive_successes = BDSHOT_OFFLINE_COUNT;
		ch->consecutive_failures = 0;
		ch->online = true;
	}

	struct BDShotTelemetry packet = {0};
	payload = (payload >> 4) & 0xFFF;
	decode_dshot_telemetry(payload, &packet);

	hrt_abstime now = hrt_absolute_time();

	switch (packet.type) {
	case DSHOT_EDT_ERPM:
		_erpms[channel].erpm = packet.value;
		_erpms[channel].ready = true;
		_erpms[channel].rate_hz = calculate_rate_hz(_erpms[channel].last_timestamp, _erpms[channel].rate_hz, now);
		_erpms[channel].last_timestamp = now;
		break;

	case DSHOT_EDT_TEMPERATURE:
		_edt_temp[channel].value = packet.value;
		_edt_temp[channel].ready = true;
		_edt_temp[channel].rate_hz = calculate_rate_hz(_edt_temp[channel].last_timestamp, _edt_temp[channel].rate_hz, now);
		_edt_temp[channel].last_timestamp = now;
		break;

	case DSHOT_EDT_VOLTAGE:
		_edt_volt[channel].value = packet.value;
		_edt_volt[channel].ready = true;
		_edt_volt[channel].rate_hz = calculate_rate_hz(_edt_volt[channel].last_timestamp, _edt_volt[channel].rate_hz, now);
		_edt_volt[channel].last_timestamp = now;
		break;

	case DSHOT_EDT_CURRENT:
		_edt_curr[channel].value = packet.value;
		_edt_curr[channel].ready = true;
		_edt_curr[channel].rate_hz = calculate_rate_hz(_edt_curr[channel].last_timestamp, _edt_curr[channel].rate_hz, now);
		_edt_curr[channel].last_timestamp = now;
		break;

	case DSHOT_EDT_STATE_EVENT:
		_edt_state[channel].value = packet.value;
		_edt_state[channel].ready = true;
		break;

	default:
		break;
	}

	_bdshot_ready_mask |= (1u << channel);
}

static float calculate_rate_hz(uint64_t last_timestamp, float last_rate_hz, uint64_t timestamp)
{
	if (last_timestamp == 0 || timestamp <= last_timestamp) {
		return last_rate_hz;
	}

	uint64_t dt_us = timestamp - last_timestamp;
	float instant_rate = 1000000.0f / (float)dt_us;

	return instant_rate * 0.125f + last_rate_hz * 0.875f;
}

static void decode_dshot_telemetry(uint32_t payload, struct BDShotTelemetry *packet)
{
	uint32_t mantissa = payload & 0x01FF;
	bool is_telemetry = !(mantissa & 0x0100);

	if (_edt_enabled && is_telemetry) {
		packet->type = (payload & 0x0F00) >> 8;
		packet->value = payload & 0x00FF;

	} else {
		uint8_t exponent = ((payload >> 9) & 0x7);
		uint16_t period = (payload & 0x1FF);
		period = period << exponent;

		packet->type = DSHOT_EDT_ERPM;

		if (period == 65408 || period == 0) {
			packet->value = 0;

		} else {
			packet->value = (1000000 * 60 / 100 + period / 2) / period;
		}
	}
}

int up_dshot_arm(bool armed)
{
	_dshot_armed = armed;

	for (uint8_t i = 0; i < MAX_DSHOT_CHANNELS; i++) {
		if (_channels[i].enabled) {
			px4_arch_gpiowrite(GPIO_OUTPUT | _channels[i].gpio, _channels[i].bdshot ? 1 : 0);
		}
	}

	return PX4_OK;
}

uint16_t up_bdshot_get_ready_mask(void)
{
	return _bdshot_ready_mask;
}

int up_bdshot_num_errors(uint8_t channel)
{
	if (channel >= MAX_DSHOT_CHANNELS) return 0;
	return _channels[channel].read_fail_crc + _channels[channel].no_response_cnt;
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	if (channel >= MAX_DSHOT_CHANNELS || !_channels[channel].bdshot) {
		return PX4_ERROR;
	}

	int status = PX4_ERROR;

	if (_channels[channel].enabled && _erpms[channel].ready) {
		*erpm = _erpms[channel].erpm;
		status = PX4_OK;
	}

	_bdshot_ready_mask &= ~(1u << channel);
	return status;
}

int up_bdshot_get_extended_telemetry(uint8_t channel, int type, uint8_t *value)
{
	if (channel >= MAX_DSHOT_CHANNELS) return PX4_ERROR;

	volatile edt_data_t *sample = NULL;

	switch (type) {
	case DSHOT_EDT_TEMPERATURE: sample = &_edt_temp[channel]; break;
	case DSHOT_EDT_VOLTAGE:     sample = &_edt_volt[channel]; break;
	case DSHOT_EDT_CURRENT:     sample = &_edt_curr[channel]; break;
	case DSHOT_EDT_STATE_EVENT: sample = &_edt_state[channel]; break;
	default: return PX4_ERROR;
	}

	if (!sample->ready) return PX4_ERROR;

	*value = sample->value;
	sample->ready = false;
	return PX4_OK;
}

int up_bdshot_channel_online(uint8_t channel)
{
	if (channel >= MAX_DSHOT_CHANNELS) return 0;
	return _channels[channel].online;
}

int up_bdshot_channel_capture_supported(uint8_t channel)
{
	if (channel >= MAX_DSHOT_CHANNELS) return 0;
	return _channels[channel].enabled && _channels[channel].bdshot;
}

void up_bdshot_status(void)
{
	PX4_INFO("ESP32-P4 DShot Driver Status:");

	if (_bdshot_channel_mask) {
		PX4_INFO("BDShot channel mask: 0x%02x", (unsigned)_bdshot_channel_mask);

		for (uint8_t i = 0; i < MAX_DSHOT_CHANNELS; i++) {
			if (_bdshot_channel_mask & (1u << i)) {
				PX4_INFO("Ch%u (GPIO %d): read_ok %lu, fail_crc %lu, no_resp %lu",
					 (unsigned)i, _channels[i].gpio,
					 (unsigned long)_channels[i].read_ok,
					 (unsigned long)_channels[i].read_fail_crc,
					 (unsigned long)_channels[i].no_response_cnt);
			}
		}
	}

	if (_edt_enabled) {
		PX4_INFO("BDShot Extended Telemetry (EDT):");

		for (uint8_t i = 0; i < MAX_DSHOT_CHANNELS; i++) {
			if (_channels[i].online) {
				PX4_INFO("Ch%u: eRPM: %dHz | Temp: %dC (%.1fHz) | Volt: %.2fV (%.1fHz) | Curr: %.1fA (%.1fHz)",
					 (unsigned)i,
					 (int)_erpms[i].rate_hz,
					 (int)_edt_temp[i].value,
					 (double)_edt_temp[i].rate_hz,
					 (double)_edt_volt[i].value * 0.25,
					 (double)_edt_volt[i].rate_hz,
					 (double)_edt_curr[i].value * 0.5,
					 (double)_edt_curr[i].rate_hz);
			}
		}
	}
}
