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
// #include "nuttx/config.h"
#include <board_config.h>
#include <systemlib/px4_macros.h>
#include <px4_platform_common/spi.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>
#include <syslog.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "espressif/esp_gpio.h"

static const px4_spi_bus_t *_spi_bus2 = nullptr;
static const px4_spi_bus_t *_spi_bus3 = nullptr;

static void spi_bus_configgpio_cs(const px4_spi_bus_t *bus)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; i++) {
		if (bus->devices[i].cs_gpio != 0) {
			px4_arch_configgpio(bus->devices[i].cs_gpio);
			/* Set CS high by default (deselected) */
			px4_arch_gpiowrite(bus->devices[i].cs_gpio & GPIO_NUM_MASK, 1);
		}
	}
}

static void esp_spixselect(const px4_spi_bus_t *bus, struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	if (!bus) {
		return;
	}

	for (int i = 0; i < SPI_BUS_MAX_DEVICES; i++) {
		if (bus->devices[i].cs_gpio != 0 && devid == bus->devices[i].devid) {
			/* Active-low chip select: write 0 to select, 1 to deselect */
			px4_arch_gpiowrite(bus->devices[i].cs_gpio & GPIO_NUM_MASK, !selected);
			break;
		}
	}
}

/****************************************************************************
 * Name: esp32p4_spiinitialize / esp_spiinitialize
 ****************************************************************************/
// __EXPORT void esp32p4_spiinitialize()
extern "C" __EXPORT void esp32p4_spiinitialize()
{
	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; i++) {
		switch (px4_spi_buses[i].bus) {
		case 2:
			_spi_bus2 = &px4_spi_buses[i];
			break;

		case 3:
			_spi_bus3 = &px4_spi_buses[i];
			break;
		}
	}

#if defined(CONFIG_ESPRESSIF_SPI2) || defined(CONFIG_ESP32P4_SPI2)
	if (_spi_bus2 && board_has_bus(BOARD_SPI_BUS, 2)) {
		syslog(LOG_DEBUG, "ESP32-P4 SPI bus configgpio cs %i\n", _spi_bus2->bus);
		spi_bus_configgpio_cs(_spi_bus2);
	}
#endif

#if defined(CONFIG_ESPRESSIF_SPI3) || defined(CONFIG_ESP32P4_SPI3)
	if (_spi_bus3 && board_has_bus(BOARD_SPI_BUS, 3)) {
		syslog(LOG_DEBUG, "ESP32-P4 SPI bus configgpio cs %i\n", _spi_bus3->bus);
		spi_bus_configgpio_cs(_spi_bus3);
	}
#endif
}

extern "C" __EXPORT void esp_spiinitialize()
{
	esp32p4_spiinitialize();
}

/****************************************************************************
 * Name: esp_spi2_select and esp_spi2_status
 *
 * Description:
 *   Called by NuttX Espressif SPI driver on bus 2.
 ****************************************************************************/
#if defined(CONFIG_ESPRESSIF_SPI2) || defined(CONFIG_ESP32P4_SPI2)

extern "C" __EXPORT void esp_spi2_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	esp_spixselect(_spi_bus2, dev, devid, selected);
}

extern "C" __EXPORT uint8_t esp_spi2_status(struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

/* Also alias with esp32p4 prefix if referenced */
// extern "C" __EXPORT void esp32p4_spi2_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
// 	__attribute__((alias("esp_spi2_select")));

// extern "C" __EXPORT uint8_t esp32p4_spi2_status(struct spi_dev_s *dev, uint32_t devid)
// 	__attribute__((alias("esp_spi2_status")));

#endif // CONFIG_ESPRESSIF_SPI2

/****************************************************************************
 * Name: esp_spi3_select and esp_spi3_status
 *
 * Description:
 *   Called by NuttX Espressif SPI driver on bus 3.
 ****************************************************************************/
#if defined(CONFIG_ESPRESSIF_SPI3) || defined(CONFIG_ESP32P4_SPI3)

extern "C" __EXPORT void esp_spi3_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	esp_spixselect(_spi_bus3, dev, devid, selected);
}

extern "C" __EXPORT uint8_t esp_spi3_status(struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

// /* Also alias with esp32p4 prefix if referenced */
// extern "C" __EXPORT void esp32p4_spi3_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
// 	__attribute__((alias("esp_spi3_select")));

// extern "C" __EXPORT uint8_t esp32p4_spi3_status(struct spi_dev_s *dev, uint32_t devid)
// 	__attribute__((alias("esp_spi3_status")));

#endif // CONFIG_ESPRESSIF_SPI3

void board_control_spi_sensors_power(bool enable_power, int bus_mask)
{
	const px4_spi_bus_t *buses = px4_spi_buses;

#if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1
	if (!buses) {
		buses = &px4_spi_buses_all_hw[0].buses[0];
	}
#endif

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (buses[bus].bus == -1) {
			break;
		}

		const bool bus_matches = bus_mask & (1 << (buses[bus].bus - 1));

		if (buses[bus].power_enable_gpio == 0 ||
		//     !board_has_bus(BOARD_SPI_BUS, buses[bus].bus) ||
		    !board_has_bus(BOARD_SPI_BUS, px4_spi_buses[bus].bus) ||
		    !bus_matches) {
			continue;
		}

		px4_arch_gpiowrite(buses[bus].power_enable_gpio & GPIO_NUM_MASK, enable_power ? 1 : 0);
	}
}

void board_control_spi_sensors_power_configgpio()
{
	const px4_spi_bus_t *buses = px4_spi_buses;

#if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1
	if (!buses) {
		buses = &px4_spi_buses_all_hw[0].buses[0];
	}
#endif

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (buses[bus].bus == -1) {
			break;
		}

		if (buses[bus].power_enable_gpio == 0 ||
		//     !board_has_bus(BOARD_SPI_BUS, buses[bus].bus)) {
		    !board_has_bus(BOARD_SPI_BUS, px4_spi_buses[bus].bus)){
			continue;
		}

		px4_arch_configgpio(buses[bus].power_enable_gpio);
	}
}

#define _PIN_OFF(def) (((def) & GPIO_NUM_MASK) | (GPIO_INPUT | GPIO_PULLDOWN))

__EXPORT void board_spi_reset(int ms, int bus_mask)
{
	bool has_power_enable = false;

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == -1) {
			break;
		}

		const bool bus_requested = bus_mask & (1 << (px4_spi_buses[bus].bus - 1));

		if (px4_spi_buses[bus].power_enable_gpio == 0 ||
		//     !board_has_bus(BOARD_SPI_BUS, buses[bus].bus) ||
		    !board_has_bus(BOARD_SPI_BUS, px4_spi_buses[bus].bus) ||
		    !bus_requested) {
			continue;
		}

		has_power_enable = true;

		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				px4_arch_configgpio(_PIN_OFF(px4_spi_buses[bus].devices[i].cs_gpio));
			}

			if (px4_spi_buses[bus].devices[i].drdy_gpio != 0) {
				px4_arch_configgpio(_PIN_OFF(px4_spi_buses[bus].devices[i].drdy_gpio));
			}
		}
	}

	if (!has_power_enable) {
		return;
	}

	/* Turn off sensors power rail */
	board_control_spi_sensors_power(false, bus_mask);
	usleep(ms * 1000);

	/* Turn back on */
	board_control_spi_sensors_power(true, bus_mask);
	usleep(100);

	/* Reconfigure CS and DRDY pins */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == -1) {
			break;
		}

		const bool bus_requested = bus_mask & (1 << (px4_spi_buses[bus].bus - 1));

		if (px4_spi_buses[bus].power_enable_gpio == 0 ||
		    !board_has_bus(BOARD_SPI_BUS, px4_spi_buses[bus].bus) ||
		    !bus_requested) {
			continue;
		}

		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				px4_arch_configgpio(px4_spi_buses[bus].devices[i].cs_gpio);
				px4_arch_gpiowrite(px4_spi_buses[bus].devices[i].cs_gpio & GPIO_NUM_MASK, 1);
			}

			if (px4_spi_buses[bus].devices[i].drdy_gpio != 0) {
				px4_arch_configgpio(px4_spi_buses[bus].devices[i].drdy_gpio);
			}
		}
	}
}
