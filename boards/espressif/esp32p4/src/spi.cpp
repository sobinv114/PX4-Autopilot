/****************************************************************************
 *
 *   Copyright (c) 2020-2026 PX4 Development Team. All rights reserved.
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
 * @file spi.cpp
 *
 * ESP32-P4 Board SPI Bus and Sensor Device Configuration
 */

#include <board_config.h>
#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

#ifndef SPI_BUS_MAX_BUS_ITEMS
#  define SPI_BUS_MAX_BUS_ITEMS 4
#endif

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {

#if defined(CONFIG_ESPRESSIF_SPI2)
	/*
	 * SPI 2: Primary internal sensor bus
	 * Pins: CLK=GPIO 30, MOSI=GPIO 29, MISO=GPIO 31, CS=GPIO 28
	 *
	 * PX4 will probe these IMU types on boot and attach to the one detected.
	 */
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO_SPI2_CS}, SPI::DRDY{-1}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42670P, SPI::CS{GPIO_SPI2_CS}, SPI::DRDY{-1}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20602,  SPI::CS{GPIO_SPI2_CS}, SPI::DRDY{-1}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42605,  SPI::CS{GPIO_SPI2_CS}, SPI::DRDY{-1}),
		initSPIDevice(DRV_BARO_DEVTYPE_BMP388,   SPI::CS{GPIO_SPI2_CS}, SPI::DRDY{-1}),
	}),
#endif

#if  defined(CONFIG_ESPRESSIF_SPI3)
	/*
	 * SPI 3: Secondary / External expansion bus
	 * Pins: CLK=GPIO 26, MOSI=GPIO 21, MISO=GPIO 27, CS=GPIO 20
	 */
	initSPIBus(SPI::Bus::SPI3, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO_SPI3_CS}, SPI::DRDY{-1}),
		initSPIDevice(DRV_BARO_DEVTYPE_MS5611,   SPI::CS{GPIO_SPI3_CS}, SPI::DRDY{-1}),
	}),
#endif

};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
