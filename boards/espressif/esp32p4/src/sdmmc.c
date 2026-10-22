/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
#include <nuttx/config.h>
#include <board_config.h>
#include <nuttx/debug.h>
#include <nuttx/arch.h>
#include <syslog.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#ifdef CONFIG_ESP32P4_SDMMC
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

/* Allow access to our custom dump function */
extern struct sdio_dev_s *sdio_initialize(int slotno);
extern void esp32p4_sdmmc_dump_details(struct sdio_dev_s *dev);
extern void sd_check(const char *tag);

#ifdef CONFIG_MMCSD

static struct sdio_dev_s *g_sdio_dev;

int esp32p4_sdio_initialize(void)
{
	int ret;

	syslog(LOG_INFO, "[SDMMC] Initializing SDIO slot 0\n");
	g_sdio_dev = sdio_initialize(0);

	if (!g_sdio_dev) {
		syslog(LOG_ERR, "[SDMMC] Failed to initialize SDIO slot 0\n");
		return -ENODEV;
	}

	syslog(LOG_INFO, "[SDMMC] SD Card SDIO slot 0 Initialized\n");

	/* Wait 50ms for SD card internal microcontroller to reach IDLE state */
	up_mdelay(100);

	syslog(LOG_INFO, "[SDMMC] Binding SDIO to the MMC/SD driver\n");
	ret = mmcsd_slotinitialize(0, g_sdio_dev);
	if (ret < 0) {
		syslog(LOG_ERR, "[SDMMC] mmcsd_slotinitialize failed: %d\n", ret);
		return ret;
	}

	syslog(LOG_INFO, "[SDMMC] SD card slot initialized successfully!\n");
	esp32p4_sdmmc_dump_details(g_sdio_dev);

	return OK;
}


// int esp32p4_sdio_initialize(void)
// {
// 	int ret;

//         syslog(LOG_INFO, "[SDMMC] Initializing SDIO slot 0\n");
// 	g_sdio_dev = sdio_initialize(0);   /* this also powers the card LDO */
// 	if (!g_sdio_dev) { return -ENODEV; }
//         syslog(LOG_INFO, "[SDMMC] SD Card SDIO slot 0 Initialized\n");
// 	/* TEST: give the card a long time to power up (SD spec needs >= 1 ms
// 	 * after VDD is stable, real cards often need far more) */
// 	usleep(1000000);                   /* 1 second */

// 	ret = mmcsd_slotinitialize(0, g_sdio_dev);
// 	if (ret < 0) { return ret; }

// 	/* Verify the card really reads, instead of assuming */
// 	uint8_t buf[512];
// 	int ok = -1;
// 	for (int i = 0; i < 5; i++) {
// 		int fd = open("/dev/mmcsd0", O_RDONLY);
// 		if (fd >= 0) {
// 			if (read(fd, buf, sizeof(buf)) == sizeof(buf)) { ok = 0; }
// 			close(fd);
// 		}
// 		if (ok == 0) { break; }
// 		usleep(200000);
// 	}
// 	syslog(LOG_ERR, "[SDMMC] read test: %s\n", ok == 0 ? "OK" : "FAILED");

// 	return OK;
// }

void sd_check(const char *tag)
{
	uint8_t buf[512];
	int ok = -1;
	int fd = open("/dev/mmcsd0", O_RDONLY);

	if (fd >= 0) {
		ok = (read(fd, buf, sizeof(buf)) == sizeof(buf)) ? 0 : -1;
		close(fd);
	}

	syslog(LOG_ERR, "[SDCHK] %s: %s\n", tag, ok == 0 ? "OK" : "FAIL");
}

#endif /* CONFIG_MMCSD */
#endif /* CONFIG_ESP32P4_SDMMC */
