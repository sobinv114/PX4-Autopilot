// /****************************************************************************
//  *
//  *   Copyright (c) 2024-2026 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  * 3. Neither the name PX4 nor the names of its contributors may be
//  *    used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/

// /**
//  * @file px4io_serial.h
//  *
//  * Serial interface for PX4IO on ESP32-P4 (NuttX POSIX UART).
//  * No STM32/i.MXRT DMA — uses /dev/ttyS* read/write.
//  */

// #pragma once

// #include <board_config.h>
// #include <px4_platform_common/px4_config.h>
// #include <px4_platform_common/micro_hal.h>
// #include <perf/perf_counter.h>
// #include <drivers/device/device.h>
// #include <modules/px4iofirmware/protocol.h>
// #include <px4_platform_common/sem.hpp>

// /**
//  * Common base used by src/drivers/px4io/px4io_serial.cpp
//  */
// class PX4IO_serial : public device::Device
// {
// public:
// 	PX4IO_serial();
// 	virtual ~PX4IO_serial();

// 	virtual int init() = 0;

// 	virtual int read(unsigned offset, void *data, unsigned count = 1) override;
// 	virtual int write(unsigned address, void *data, unsigned count = 1) override;

// protected:
// 	/**
// 	 * Base init — stores packet buffer and creates bus semaphore.
// 	 */
// 	int init(IOPacket *io_buffer);

// 	/**
// 	 * Arch must implement: send packet, wait for reply, return OK or -errno.
// 	 */
// 	virtual int _bus_exchange(IOPacket *_packet) = 0;

// 	perf_counter_t _pc_txns;
// 	perf_counter_t _pc_retries;
// 	perf_counter_t _pc_timeouts;
// 	perf_counter_t _pc_crcerrs;
// 	perf_counter_t _pc_protoerrs;
// 	perf_counter_t _pc_uerrs;
// 	perf_counter_t _pc_idle;
// 	perf_counter_t _pc_badidle;

// private:
// 	IOPacket *_io_buffer_ptr{nullptr};
// 	px4_sem_t _bus_semaphore;

// 	PX4IO_serial(const PX4IO_serial &) = delete;
// 	PX4IO_serial &operator=(const PX4IO_serial &) = delete;
// };

// /**
//  * ESP32-P4 implementation: NuttX serial device (POSIX).
//  */
// class ArchPX4IOSerial : public PX4IO_serial
// {
// public:
// 	ArchPX4IOSerial();
// 	ArchPX4IOSerial(const ArchPX4IOSerial &) = delete;
// 	ArchPX4IOSerial &operator=(const ArchPX4IOSerial &) = delete;
// 	~ArchPX4IOSerial() override;

// 	int init() override;
// 	int ioctl(unsigned operation, unsigned &arg) override;

// protected:
// 	int _bus_exchange(IOPacket *_packet) override;

// private:
// 	int _fd{-1};
// 	IOPacket _io_buffer{};

// 	/** Max time to wait for a full reply (ms). PX4IO is ~0.5 ms at 1.5 Mbps. */
// 	static constexpr unsigned EXCHANGE_TIMEOUT_MS = 10;
// };
















/**
 * @file px4io_serial.h
 *
 * Serial interface definition for PX4IO (ESP32-P4, NuttX tty based)
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include <board_config.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>

#include <perf/perf_counter.h>

#include <drivers/device/device.h>
#include <modules/px4iofirmware/protocol.h>

class PX4IO_serial : public device::Device
{
public:
	PX4IO_serial();
	virtual ~PX4IO_serial();

	virtual int	init() = 0;
	virtual int	read(unsigned offset, void *data, unsigned count = 1);
	virtual int	write(unsigned address, void *data, unsigned count = 1);

protected:
	/**
	 * Does the PX4IO_serial instance initialization.
	 * @param io_buffer The IO buffer that should be used for transfers.
	 * @return 0 on success.
	 */
	int		init(IOPacket *io_buffer);

	/**
	 * Start the transaction with IO and wait for it to complete.
	 */
	virtual int	_bus_exchange(IOPacket *_packet) = 0;

	/**
	 * Performance counters.
	 */
	perf_counter_t		_pc_txns;
	perf_counter_t		_pc_retries;
	perf_counter_t		_pc_timeouts;
	perf_counter_t		_pc_crcerrs;
	perf_counter_t		_pc_protoerrs;
	perf_counter_t		_pc_uerrs;
	perf_counter_t		_pc_idle;
	perf_counter_t		_pc_badidle;

private:
	IOPacket		*_io_buffer_ptr;

	/** bus-ownership lock */
	px4_sem_t		_bus_semaphore;

	/* do not allow copying this class */
	PX4IO_serial(PX4IO_serial &);
	PX4IO_serial &operator = (const PX4IO_serial &);
};


class ArchPX4IOSerial : public PX4IO_serial
{
public:
	ArchPX4IOSerial();
	ArchPX4IOSerial(ArchPX4IOSerial &) = delete;
	ArchPX4IOSerial &operator = (const ArchPX4IOSerial &) = delete;
	virtual ~ArchPX4IOSerial();

	virtual int	init();
	virtual int	ioctl(unsigned operation, unsigned &arg);

protected:
	/**
	 * Start the transaction with IO and wait for it to complete.
	 */
	int		_bus_exchange(IOPacket *_packet);

private:
	static constexpr size_t kBufAlign = 64;
	static constexpr size_t kBufSize  = (sizeof(IOPacket) + kBufAlign - 1) & ~(kBufAlign - 1);

	/** discard any bytes waiting in the receive queue */
	void	_drain_rx();

	/** write len bytes before the deadline */
	int	_write_all(const uint8_t *buf, size_t len, const struct timespec &deadline);

	/** read exactly len bytes before the deadline */
	int	_read_exact(uint8_t *buf, size_t len, const struct timespec &deadline);

	int		_fd{-1};
	IOPacket	*_current_packet{nullptr};

	/** IO buffer storage */
	static uint8_t _io_buffer_storage[kBufSize] __attribute__((aligned(kBufAlign)));
};
