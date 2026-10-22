/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file DatamanClient.cpp
 * Direct Hardware-Optimized Dataman Client & Cache Implementation for ESP32-P4
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#ifndef CONFIG_NUM_MISSION_ITEMS_SUPPORTED
#define CONFIG_NUM_MISSION_ITEMS_SUPPORTED 1000
#endif

#include <dataman_client/DatamanClient.hpp>
#include <dataman/dataman.h>
#include <stdlib.h>
#include <cstring>

/* --------------------------------------------------------------------------
 * Dynamic PSRAM Storage (0 bytes static SRAM consumption in .dram0.bss)
 * -------------------------------------------------------------------------- */
static uint8_t *s_dataman_ram = nullptr;
static size_t s_dataman_ram_size = 0;
static unsigned int s_key_offsets[DM_KEY_NUM_KEYS];

static void init_storage_once()
{
	if (s_dataman_ram == nullptr) {
		s_key_offsets[0] = 0;

		for (int i = 0; i < ((int)DM_KEY_NUM_KEYS - 1); i++) {
			s_key_offsets[i + 1] = s_key_offsets[i] + (g_per_item_max_index[i] * (g_per_item_size[i] + 4));
		}

		s_dataman_ram_size = s_key_offsets[DM_KEY_NUM_KEYS - 1] +
				     (g_per_item_max_index[DM_KEY_NUM_KEYS - 1] * (g_per_item_size[DM_KEY_NUM_KEYS - 1] + 4));

		s_dataman_ram = (uint8_t *)malloc(s_dataman_ram_size);

		if (s_dataman_ram != nullptr) {
			memset(s_dataman_ram, 0, s_dataman_ram_size);
		} else {
			PX4_ERR("Failed to allocate dataman RAM in PSRAM (%zu bytes)", s_dataman_ram_size);
		}
	}
}

/* --------------------------------------------------------------------------
 * DatamanClient Implementation
 * -------------------------------------------------------------------------- */

DatamanClient::DatamanClient()
{
	_sync_perf = perf_alloc(PC_ELAPSED, "DatamanClient: sync");
	init_storage_once();
	_client_id = 1;
}

DatamanClient::~DatamanClient()
{
	perf_free(_sync_perf);
}

void DatamanClient::clearPendingResponse()
{
}

bool DatamanClient::readSync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout)
{
	init_storage_once();

	if (s_dataman_ram == nullptr || item >= DM_KEY_NUM_KEYS || index >= g_per_item_max_index[item]) {
		return false;
	}

	if (length > g_per_item_size[item]) {
		PX4_ERR("Length %" PRIu32 " can't fit in data size for item %" PRIi8, length, static_cast<uint8_t>(item));
		return false;
	}

	size_t offset = s_key_offsets[item] + (index * (g_per_item_size[item] + 4));

	if (offset + length + 4 > s_dataman_ram_size) {
		return false;
	}

	perf_begin(_sync_perf);
	uint8_t *slot = &s_dataman_ram[offset];
	uint8_t len = slot[0];

	if (len > 0) {
		uint32_t copy_len = (len < length) ? len : length;
		memcpy(buffer, slot + 4, copy_len);
	} else {
		memset(buffer, 0, length);
	}
	perf_end(_sync_perf);

	return true;
}

bool DatamanClient::writeSync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout)
{
	init_storage_once();

	if (s_dataman_ram == nullptr || item >= DM_KEY_NUM_KEYS || index >= g_per_item_max_index[item]) {
		return false;
	}

	if (length > g_per_item_size[item]) {
		PX4_ERR("Length %" PRIu32 " can't fit in data size for item %" PRIi8, length, static_cast<uint8_t>(item));
		return false;
	}

	size_t offset = s_key_offsets[item] + (index * (g_per_item_size[item] + 4));

	if (offset + length + 4 > s_dataman_ram_size) {
		return false;
	}

	perf_begin(_sync_perf);
	uint8_t *slot = &s_dataman_ram[offset];
	slot[0] = static_cast<uint8_t>(length);
	memcpy(slot + 4, buffer, length);
	perf_end(_sync_perf);

	return true;
}

bool DatamanClient::clearSync(dm_item_t item, hrt_abstime timeout)
{
	init_storage_once();

	if (s_dataman_ram == nullptr || item >= DM_KEY_NUM_KEYS) {
		return false;
	}

	perf_begin(_sync_perf);
	size_t offset = s_key_offsets[item];
	for (unsigned i = 0; i < g_per_item_max_index[item]; i++) {
		s_dataman_ram[offset] = 0;
		offset += (g_per_item_size[item] + 4);
	}
	perf_end(_sync_perf);

	return true;
}

bool DatamanClient::readAsync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length)
{
	if (_state != State::Idle) {
		return false;
	}

	if (length > g_per_item_size[item]) {
		PX4_ERR("Length %" PRIu32 " can't fit in data size for item %" PRIi8, length, static_cast<uint8_t>(item));
		return false;
	}

	_active_request.timestamp = hrt_absolute_time();
	_active_request.request_type = DM_READ;
	_active_request.item = item;
	_active_request.index = index;
	_active_request.buffer = buffer;
	_active_request.length = length;

	_state = State::RequestSent;
	return true;
}

bool DatamanClient::writeAsync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length)
{
	if (_state != State::Idle) {
		return false;
	}

	if (length > g_per_item_size[item]) {
		PX4_ERR("Length %" PRIu32 " can't fit in data size for item %" PRIi8, length, static_cast<uint8_t>(item));
		return false;
	}

	_active_request.timestamp = hrt_absolute_time();
	_active_request.request_type = DM_WRITE;
	_active_request.item = item;
	_active_request.index = index;
	_active_request.buffer = buffer;
	_active_request.length = length;

	_state = State::RequestSent;
	return true;
}

bool DatamanClient::clearAsync(dm_item_t item)
{
	if (_state != State::Idle) {
		return false;
	}

	_active_request.timestamp = hrt_absolute_time();
	_active_request.request_type = DM_CLEAR;
	_active_request.item = item;
	_active_request.index = 0;
	_active_request.buffer = nullptr;
	_active_request.length = 0;

	_state = State::RequestSent;
	return true;
}

void DatamanClient::update()
{
	if (_state == State::RequestSent) {
		bool op_ok = false;

		switch (_active_request.request_type) {
		case DM_READ:
			op_ok = readSync(_active_request.item, _active_request.index, _active_request.buffer, _active_request.length, 0);
			break;

		case DM_WRITE:
			op_ok = writeSync(_active_request.item, _active_request.index, _active_request.buffer, _active_request.length, 0);
			break;

		case DM_CLEAR:
			op_ok = clearSync(_active_request.item, 0);
			break;

		default:
			op_ok = false;
			break;
		}

		_response_status = op_ok ? dataman_response_s::STATUS_SUCCESS : dataman_response_s::STATUS_FAILURE_NO_DATA;
		_state = State::ResponseReceived;
	}
}

bool DatamanClient::lastOperationCompleted(bool &success)
{
	bool completed = false;
	success = false;

	if (_state == State::ResponseReceived) {
		if (_response_status == dataman_response_s::STATUS_SUCCESS) {
			success = true;
		}
		_state = State::Idle;
		completed = true;
	}

	return completed;
}

void DatamanClient::abortCurrentOperation()
{
	_state = State::Idle;
}

/* --------------------------------------------------------------------------
 * DatamanCache Implementation
 * -------------------------------------------------------------------------- */

DatamanCache::DatamanCache(const char *cache_miss_perf_counter_name, uint32_t num_items)
	: _cache_miss_perf(perf_alloc(PC_COUNT, cache_miss_perf_counter_name))
{
	if (num_items == 0) {
		return;
	}

	_items = new Item[num_items]{};

	if (_items != nullptr) {
		_num_items = num_items;
	} else {
		PX4_ERR("alloc failed");
	}
}

DatamanCache::~DatamanCache()
{
	_client.abortCurrentOperation();
	delete[] _items;
	perf_free(_cache_miss_perf);
}

void DatamanCache::resize(uint32_t num_items)
{
	if (num_items == 0) {
		_client.abortCurrentOperation();
		delete[] _items;
		_items = nullptr;
		resetCacheState();
		_num_items = 0;
		return;
	}

	Item *new_items = new Item[num_items]{};

	if (new_items != nullptr) {
		_client.abortCurrentOperation();

		uint32_t num_min = num_items < _num_items ? num_items : _num_items;

		for (uint32_t i = 0; i < num_min; ++i) {
			new_items[i] = _items[i];

			if (new_items[i].cache_state == State::RequestSent) {
				new_items[i].cache_state = State::RequestPrepared;
			}
		}

		delete[] _items;
		_items = new_items;
		_num_items = num_items;

		_load_index = (_load_index < _num_items) ? _load_index : 0;
		_update_index = (_update_index < _num_items) ? _update_index : 0;
		_item_counter = (_item_counter < _num_items) ? _item_counter : _num_items;
	} else {
		PX4_ERR("alloc failed");
	}
}

bool DatamanCache::load(dm_item_t item, uint32_t index)
{
	if (!_items) {
		return false;
	}

	bool success = false;
	bool duplicate = false;

	for (uint32_t i = 0; i < _num_items; ++i) {
		if (_items[i].cache_state != State::Idle &&
		    _items[i].cache_state != State::Error &&
		    _items[i].response.item == item &&
		    _items[i].response.index == index) {
			duplicate = true;
			break;
		}
	}

	if (!duplicate && (_item_counter < _num_items)) {
		_items[_load_index].cache_state = State::RequestPrepared;
		_items[_load_index].response.item = item;
		_items[_load_index].response.index = index;

		_load_index = (_load_index + 1) % _num_items;
		++_item_counter;
		success = true;
	}

	return success;
}

bool DatamanCache::loadWait(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout)
{
	if (length > g_per_item_size[item]) {
		PX4_ERR("Length %" PRIu32 " can't fit in data size for item %" PRIi8, length, static_cast<uint8_t>(item));
		return false;
	}

	bool success = false;
	bool item_found = false;

	if (_items) {
		for (uint32_t i = 0; i < _num_items; ++i) {
			if ((_items[i].response.item == item) &&
			    (_items[i].response.index == index)) {
				item_found = true;

				if (_items[i].cache_state == State::ResponseReceived) {
					memcpy(buffer, _items[i].response.data, length);
					success = true;
					break;
				}
			}
		}
	}

	if (!success) {
		perf_count(_cache_miss_perf);
	}

	if (!success) {
		success = _client.readSync(item, index, buffer, length, timeout);

		if (success && !item_found && _items && (_item_counter < _num_items)) {
			_items[_load_index].cache_state = State::ResponseReceived;
			_items[_load_index].response.item = item;
			_items[_load_index].response.index = index;
			memcpy(_items[_load_index].response.data, buffer, length);

			_load_index = (_load_index + 1) % _num_items;
			++_item_counter;
		}
	}

	return success;
}

bool DatamanCache::writeWait(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout)
{
	if (length > g_per_item_size[item]) {
		PX4_ERR("Length %" PRIu32 " can't fit in data size for item %" PRIi8, length, static_cast<uint8_t>(item));
		return false;
	}

	bool success = _client.writeSync(item, index, buffer, length, timeout);

	if (success && _items) {
		for (uint32_t i = 0; i < _num_items; ++i) {
			if ((_items[i].response.item == item) &&
			    (_items[i].response.index == index) &&
			    ((_items[i].cache_state == State::ResponseReceived) ||
			     (_items[i].cache_state == State::RequestPrepared))) {

				memcpy(_items[i].response.data, buffer, length);
				_items[i].cache_state = State::ResponseReceived;
				break;
			}
		}
	}

	return success;
}

bool DatamanCache::updateCachedItem(dm_item_t item, uint32_t index, const uint8_t *buffer, uint32_t length)
{
	if (!_items || buffer == nullptr || _num_items == 0) {
		return false;
	}

	if (length > g_per_item_size[item]) {
		PX4_ERR("Length %" PRIu32 " can't fit in data size for item %" PRIi8, length, static_cast<uint8_t>(item));
		return false;
	}

	for (uint32_t i = 0; i < _num_items; ++i) {
		if ((_items[i].response.item == item) && (_items[i].response.index == index)
		    && (_items[i].cache_state == State::ResponseReceived)) {
			memcpy(_items[i].response.data, buffer, length);
			return true;
		}
	}

	return false;
}

void DatamanCache::update()
{
	if (_item_counter > 0) {
		_client.update();

		bool success = false;
		bool response_success = false;

		switch (_items[_update_index].cache_state) {
		case State::Idle:
			break;

		case State::ResponseReceived:
			changeUpdateIndex();
			break;

		case State::RequestPrepared:
			success = _client.readAsync(static_cast<dm_item_t>(_items[_update_index].response.item),
						    _items[_update_index].response.index,
						    _items[_update_index].response.data,
						    g_per_item_size[_items[_update_index].response.item]);

			if (success) {
				_items[_update_index].cache_state = State::RequestSent;
			} else {
				_items[_update_index].cache_state = State::Error;
			}
			break;

		case State::RequestSent:
			if (_client.lastOperationCompleted(response_success)) {
				if (response_success) {
					_items[_update_index].cache_state = State::ResponseReceived;
					changeUpdateIndex();
				} else {
					_items[_update_index].cache_state = State::Error;
				}
			}
			break;

		case State::Error:
			break;
		}

		if (_items[_update_index].cache_state == State::Error) {
			PX4_ERR("Caching: item %" PRIu8 ", index %" PRIu32 ", status %" PRIu8,
				_items[_update_index].response.item, _items[_update_index].response.index,
				_items[_update_index].response.status);
			changeUpdateIndex();
		}
	}
}

void DatamanCache::resetCacheState()
{
	_update_index = 0;
	_item_counter = 0;
	_load_index = 0;
}

void DatamanCache::invalidate()
{
	for (uint32_t i = 0; i < _num_items; ++i) {
		_items[i].cache_state = State::Idle;
	}

	resetCacheState();
	_client.abortCurrentOperation();
}

inline void DatamanCache::changeUpdateIndex()
{
	if (_num_items == 0) {
		_update_index = 0;
		_item_counter = 0;
		return;
	}

	_update_index = (_update_index + 1) % _num_items;

	if (_item_counter > 0) {
		--_item_counter;
	}
}

