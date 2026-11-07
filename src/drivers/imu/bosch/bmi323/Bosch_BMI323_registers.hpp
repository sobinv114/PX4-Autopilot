/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file Bosch_BMI323_registers.hpp
 *
 * Bosch BMI323 registers (datasheet BST-BMI323-DS000-13).
 *
 * All BMI323 registers are 16 bit wide, transmitted LSB first.
 * SPI read: [addr|0x80][dummy][LSB][MSB]...   SPI write: [addr][LSB][MSB]...
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace Bosch_BMI323
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz max (datasheet table 41)
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t chip_id = 0x43; // CHIP_ID[7:0]

enum class Register : uint8_t {
	CHIP_ID         = 0x00,
	ERR_REG         = 0x01,
	STATUS          = 0x02,
	ACC_DATA_X      = 0x03,
	GYR_DATA_X      = 0x06,
	TEMP_DATA       = 0x09,
	INT_STATUS_INT1 = 0x0D,
	FEATURE_IO1     = 0x11,
	FEATURE_IO2     = 0x12,
	FEATURE_IO_STATUS = 0x14,
	FIFO_FILL_LEVEL = 0x15, // in 16 bit words, 11 bit
	FIFO_DATA       = 0x16, // burst read only, address does not increment

	ACC_CONF        = 0x20,
	GYR_CONF        = 0x21,

	FIFO_WATERMARK  = 0x35, // in 16 bit words, 10 bit
	FIFO_CONF       = 0x36,
	FIFO_CTRL       = 0x37,
	IO_INT_CTRL     = 0x38,
	INT_CONF        = 0x39,
	INT_MAP1        = 0x3A,
	INT_MAP2        = 0x3B,

	FEATURE_CTRL      = 0x40,
	FEATURE_DATA_ADDR = 0x41,
	FEATURE_DATA_TX   = 0x42,

	CMD             = 0x7E,
	CFG_RES         = 0x7F,
};

// CMD
static constexpr uint16_t CMD_SOFTRESET = 0xDEAF;

// Feature engine enable + configuration array upload (datasheet 5.8.1, Bosch SensorAPI
// bmi3_configure_enhanced_flexibility). Optional: only needed e.g. for step counter watermark > 65535.
static constexpr uint16_t FEATURE_IO2_ENGINE_KEY    = 0x012C; // written to FEATURE_IO2 before enabling
static constexpr uint16_t FEATURE_IO_STATUS_TRIGGER = 0x0001;
static constexpr uint16_t FEATURE_CTRL_ENGINE_EN    = 0x0001;
static constexpr uint16_t FEATURE_IO1_ERROR_STATUS_MASK = 0x000F;
static constexpr uint16_t FEATURE_IO1_ENGINE_ACTIVE = 0x1; // "feature engine activated"
static constexpr uint16_t FEATURE_IO1_CONFIG_FAILED = 0x3; // "configuration string download failed"
static constexpr uint16_t CMD_CFG_UNLOCK_1 = 0xD3AC; // written first
static constexpr uint16_t CMD_CFG_UNLOCK_2 = 0x64AD; // written second
static constexpr uint16_t CFG_RES_UPLOAD_PAGE = 0x8007; // CFG_RES bytes {0x07, 0x80}
static constexpr uint8_t  CFG_RES_UNLOCKED_MSB = 0x80;

// ERR_REG
namespace ERR_REG_BIT
{
static constexpr uint16_t fatal_err      = (1u << 0);
static constexpr uint16_t feat_eng_ovrld = (1u << 2);
static constexpr uint16_t feat_eng_wd    = (1u << 4);
static constexpr uint16_t acc_conf_err   = (1u << 5);
static constexpr uint16_t gyr_conf_err   = (1u << 6);
static constexpr uint16_t i3c_error0     = (1u << 8);
static constexpr uint16_t i3c_error3     = (1u << 11);
}

// ACC_CONF / GYR_CONF layout (identical):
// [14:12] mode  [10:8] avg_num  [7] bw  [6:4] range  [3:0] odr
namespace CONF
{
static constexpr uint16_t MODE_DISABLED = (0x0u << 12);
static constexpr uint16_t MODE_LOW_PWR  = (0x3u << 12);
static constexpr uint16_t MODE_NORMAL   = (0x4u << 12);
static constexpr uint16_t MODE_HIGH_PERF = (0x7u << 12);

static constexpr uint16_t BW_ODR_2 = (0u << 7); // -3dB = ODR/2
static constexpr uint16_t BW_ODR_4 = (1u << 7); // -3dB = ODR/4

static constexpr uint16_t ODR_800  = 0xB;
static constexpr uint16_t ODR_1600 = 0xC;
static constexpr uint16_t ODR_3200 = 0xD;
static constexpr uint16_t ODR_6400 = 0xE;

// bits that are writable/defined (bit 15 and bit 11 reserved)
static constexpr uint16_t MASK = 0x77FF;
}

namespace ACC_RANGE
{
static constexpr uint16_t G2  = (0u << 4);
static constexpr uint16_t G4  = (1u << 4);
static constexpr uint16_t G8  = (2u << 4);
static constexpr uint16_t G16 = (3u << 4);
static constexpr uint16_t MASK = (7u << 4);
}

namespace GYR_RANGE
{
static constexpr uint16_t DPS125  = (0u << 4);
static constexpr uint16_t DPS250  = (1u << 4);
static constexpr uint16_t DPS500  = (2u << 4);
static constexpr uint16_t DPS1000 = (3u << 4);
static constexpr uint16_t DPS2000 = (4u << 4);
static constexpr uint16_t MASK = (7u << 4);
}

// FIFO_CONF
namespace FIFO_CONF_BIT
{
static constexpr uint16_t fifo_stop_on_full = (1u << 0);
static constexpr uint16_t fifo_time_en      = (1u << 8);
static constexpr uint16_t fifo_acc_en       = (1u << 9);
static constexpr uint16_t fifo_gyr_en       = (1u << 10);
static constexpr uint16_t fifo_temp_en      = (1u << 11);
}

// FIFO_CTRL
static constexpr uint16_t FIFO_CTRL_FLUSH = (1u << 0);

// IO_INT_CTRL
namespace IO_INT_CTRL_BIT
{
static constexpr uint16_t int1_lvl_high   = (1u << 0); // 1 = active high
static constexpr uint16_t int1_od         = (1u << 1); // 1 = open drain
static constexpr uint16_t int1_output_en  = (1u << 2);
}

// INT_CONF
static constexpr uint16_t INT_CONF_LATCH = (1u << 0);

// INT_MAP2: 2 bit fields, 0 = off, 1 = INT1, 2 = INT2, 3 = IBI
namespace INT_MAP2_BIT
{
static constexpr uint16_t temp_drdy_int1     = (1u << 6);
static constexpr uint16_t gyr_drdy_int1      = (1u << 8);
static constexpr uint16_t acc_drdy_int1      = (1u << 10);
static constexpr uint16_t fifo_wm_int1       = (1u << 12);
static constexpr uint16_t fifo_full_int1     = (1u << 14);

// every mapping field touched by this driver (err_status/tap/i3c untouched)
static constexpr uint16_t drdy_fifo_fields_mask = 0xFFC0;
}

namespace FIFO
{
// buffer size is 2048 bytes
static constexpr size_t SIZE_WORDS = 1024;

// Frames have NO header. With accel+gyro enabled a frame is
// [ax ay az gx gy gz] = 6 words = 12 bytes (order fixed: acc, gyr, temp, time)
static constexpr size_t FRAME_WORDS = 6;
static constexpr size_t FRAME_BYTES = FRAME_WORDS * 2;

// signature of invalid (settling) frames, first word
static constexpr uint16_t DUMMY_ACC_WORD0 = 0x7F01;
static constexpr uint16_t DUMMY_GYR_WORD0 = 0x7F02;
static constexpr uint16_t INVALID         = 0x8000;

// one axis triplet as stored in the FIFO (3 little-endian 16 bit words)
struct Data {
	uint8_t x_lsb;
	uint8_t x_msb;
	uint8_t y_lsb;
	uint8_t y_msb;
	uint8_t z_lsb;
	uint8_t z_msb;
};

// accel + gyro frame (FIFO_CONF: acc_en + gyr_en, no temp, no time). NO header byte.
struct Frame {
	Data accel;
	Data gyro;
};
static_assert(sizeof(Data) == 6 && sizeof(Frame) == FRAME_BYTES, "FIFO frame layout");
} // namespace FIFO

} // namespace Bosch_BMI323
