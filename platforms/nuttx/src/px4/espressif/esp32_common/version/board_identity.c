/****************************************************************************
 *
 *   Copyright (C) 2017-2026 PX4 Development Team. All rights reserved.
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
 * @file board_identity.c
 * Implementation of ESP32-P4 unique Board identity API using eFuse MAC
 */

#include <px4_platform_common/px4_config.h>
#include <stdio.h>
#include <string.h>

#include "esp_mac.h"

#define CPU_UUID_BYTE_FORMAT_ORDER          {3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8}
#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00ff0000) >> 8) | (((x) & 0x0000ff00) << 8) | ((x) << 24))

typedef const uint8_t uuid_uint8_reorder_t[PX4_CPU_UUID_BYTE_LENGTH];

void board_get_uuid(uuid_byte_t uuid_bytes)
{
	uuid_uint8_reorder_t reorder = CPU_UUID_BYTE_FORMAT_ORDER;
	union {
		uuid_byte_t b;
		uuid_uint32_t w;
	} id;

	board_get_uuid32(id.w);

	for (int i = 0; i < PX4_CPU_UUID_BYTE_LENGTH; i++) {
		uuid_bytes[i] = id.b[reorder[i]];
	}
}

__EXPORT void board_get_uuid32(uuid_uint32_t uuid_words)
{
	uint8_t mac[6] = {0};

	if (esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
		/* Populate 3 x 32-bit words (12 bytes) using real eFuse MAC */
		uuid_words[0] = (mac[0] << 24) | (mac[1] << 16) | (mac[2] << 8) | mac[3];
		uuid_words[1] = (mac[4] << 24) | (mac[5] << 16) | 0xAA55;
		uuid_words[2] = 0x50340001; /* Identifier tag "P4\0\1" */
	} else {
		for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
			uuid_words[i] = i;
		}
	}
}

int board_get_uuid32_formated(char *format_buffer, int size,
			      const char *format,
			      const char *seperator)
{
	uuid_uint32_t uuid;
	board_get_uuid32(uuid);
	int offset = 0;
	int sep_size = seperator ? strlen(seperator) : 0;

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, format, uuid[i]);

		if (sep_size && i < PX4_CPU_UUID_WORD32_LENGTH - 1) {
			strcat(&format_buffer[offset], seperator);
			offset += sep_size;
		}
	}

	return 0;
}

int board_get_mfguid(mfguid_t mfgid)
{
	uint8_t mac[6] = {0};
	memset(mfgid, 0, PX4_CPU_MFGUID_BYTE_LENGTH);

	if (esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
		memcpy(mfgid, mac, sizeof(mac));
	}

	return PX4_CPU_MFGUID_BYTE_LENGTH;
}

int board_get_mfguid_formated(char *format_buffer, int size)
{
	mfguid_t mfguid;
	board_get_mfguid(mfguid);
	int offset = 0;

	for (unsigned i = 0; offset < size && i < PX4_CPU_MFGUID_BYTE_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", mfguid[i]);
	}

	return offset;
}

int board_get_px4_guid(px4_guid_t px4_guid)
{
	memset(px4_guid, 0, PX4_GUID_BYTE_LENGTH);

	uint8_t mac[6] = {0};
	if (esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
		memcpy(px4_guid, mac, sizeof(mac));
	}

	return PX4_GUID_BYTE_LENGTH;
}

int board_get_px4_guid_formated(char *format_buffer, int size)
{
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);

	int offset = 0;
	size = (size & 1) ? size : (size - 1);

	for (unsigned i = PX4_GUID_BYTE_LENGTH - size / 2; offset < size && i < PX4_GUID_BYTE_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", px4_guid[i]);
	}

	return offset;
}
