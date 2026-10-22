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
#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#ifndef CONFIG_NUM_MISSION_ITEMS_SUPPORTED
#define CONFIG_NUM_MISSION_ITEMS_SUPPORTED 250
#endif

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/dataman_request.h>
#include <uORB/topics/dataman_response.h>
#include <dataman/dataman.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/posix.h>

using namespace time_literals;

class DatamanClientTestPeer;
class DatamanCacheTestPeer;

class DatamanClient
{
public:
	DatamanClient();
	~DatamanClient();

	DatamanClient(const DatamanClient &) = delete;
	DatamanClient &operator=(const DatamanClient &) = delete;

	/**
	 * @brief Reads data synchronously from the dataman for the specified item and index.
	 */
	bool readSync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout = 200_ms);

	/**
	 * @brief Write data to the dataman synchronously.
	 */
	bool writeSync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout = 500_ms);

	/**
	 * @brief Clears the data in the specified dataman item.
	 */
	bool clearSync(dm_item_t item, hrt_abstime timeout = 500_ms);

	/**
	 * @brief Initiates an asynchronous request to read data from dataman.
	 */
	bool readAsync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length);

	/**
	 * @brief Initiates an asynchronous request to write data to dataman.
	 */
	bool writeAsync(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length);

	/**
	 * @brief Initiates an asynchronous request to clear an item in dataman.
	 */
	bool clearAsync(dm_item_t item);

	/**
	 * @brief Updates the state of the dataman client for asynchronous functions.
	 */
	void update();

	/**
	 * @brief Check if the last dataman operation has completed and whether it was successful.
	 */
	bool lastOperationCompleted(bool &success);

	/**
	 * @brief Abort any async operation currently in progress.
	 */
	void abortCurrentOperation();

	/**
	 * @brief Get subscription fd used for Dataman responses.
	 */
	orb_sub_t responseSubscription() const { return ORB_SUB_INVALID; }

private:
	friend class DatamanClientTestPeer;

	enum class State {
		Idle,
		RequestSent,
		ResponseReceived
	};

	struct Request {
		hrt_abstime timestamp;
		dm_function_t request_type;
		dm_item_t item;
		uint32_t index;
		uint8_t *buffer;
		uint32_t length;
	};

	void clearPendingResponse();

	State _state{State::Idle};
	Request _active_request{};
	uint8_t _response_status{dataman_response_s::STATUS_SUCCESS};

	static constexpr uint8_t CLIENT_ID_NOT_SET{0};
	uint8_t _client_id{1}; // Direct valid client ID

	perf_counter_t _sync_perf{nullptr};
};

class DatamanCache
{
public:
	DatamanCache(const char *cache_miss_perf_counter_name, uint32_t num_items);
	~DatamanCache();

	void resize(uint32_t num_items);
	void invalidate();
	bool load(dm_item_t item, uint32_t index);
	bool loadWait(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout = 0);
	bool writeWait(dm_item_t item, uint32_t index, uint8_t *buffer, uint32_t length, hrt_abstime timeout = 5000_ms);
	bool updateCachedItem(dm_item_t item, uint32_t index, const uint8_t *buffer, uint32_t length);
	void update();
	bool isLoading() const { return (_item_counter > 0); }
	DatamanClient &client() { return _client; }
	int size() const { return _num_items; }

private:
	friend class DatamanCacheTestPeer;

	enum class State {
		Idle,
		RequestPrepared,
		RequestSent,
		ResponseReceived,
		Error
	};

	struct Item {
		dataman_response_s response;
		State cache_state{State::Idle};
	};

	void resetCacheState();
	inline void changeUpdateIndex();

	Item *_items{nullptr};
	uint32_t _load_index{0};
	uint32_t _update_index{0};
	uint32_t _item_counter{0};
	uint32_t _num_items{0};

	DatamanClient _client{};
	perf_counter_t _cache_miss_perf{nullptr};
};


