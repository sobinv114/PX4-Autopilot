/****************************************************************************
 * boards/espressif/esp32p4/src/can.c
 *
 * Board-specific CAN (TWAI) functions for ESP32-P4 in PX4.
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/config.h>
#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "board_config.h"

#if defined(CONFIG_CAN) && defined(CONFIG_ESPRESSIF_TWAI)
#  include "espressif/esp_twai.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
// extern int esp_rom_printf(const char *fmt, ...);


#if defined(CONFIG_CAN) && defined(CONFIG_ESPRESSIF_TWAI)

int can_devinit(void)
{
	static bool initialized = false;
	int ret = OK;

	if (initialized) {
		return OK;
	}

#if defined(CONFIG_ESPRESSIF_TWAI0)
	{
		struct can_dev_s *twai0 = esp_twaiinitialize(0);

		if (twai0 == NULL) {
			canerr("ERROR: Failed to initialize TWAI0\n");
		} else {
			ret = can_register("/dev/can0", twai0);

			if (ret < 0) {
				canerr("ERROR: can_register /dev/can0 failed: %d\n", ret);
			} else {
				caninfo("TWAI0 registered at /dev/can0\n");
				syslog(LOG_INFO, "TWAI0 registered at /dev/can0\n");
				// esp_rom_printf("TWAI0 registered at /dev/can0\n");
			}
		}
	}
#endif

#if defined(CONFIG_ESPRESSIF_TWAI1)
	{
		struct can_dev_s *twai1 = esp_twaiinitialize(1);

		if (twai1 == NULL) {
			canerr("ERROR: Failed to initialize TWAI1\n");
		} else {
			ret = can_register("/dev/can1", twai1);

			if (ret < 0) {
				canerr("ERROR: can_register /dev/can1 failed: %d\n", ret);
			} else {
				caninfo("TWAI1 registered at /dev/can1\n");
				syslog(LOG_INFO, "TWAI0 registered at /dev/can1\n");
				// esp_rom_printf("TWAI0 registered at /dev/can1\n");
			}
		}
	}
#endif

	initialized = true;
	return ret;
}

#else

int can_devinit(void)
{
	return -ENODEV;
}

#endif /* CONFIG_CAN && CONFIG_ESPRESSIF_TWAI */
