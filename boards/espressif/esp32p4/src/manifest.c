/****************************************************************************
 *
 *   Copyright (c) 2018-2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file manifest.c
 *
 * Hardware manifest query interface for ESP32-P4.
 */

#include <nuttx/config.h>
#include <board_config.h>

#include <inttypes.h>
#include <stdbool.h>
#include <syslog.h>

#include "systemlib/px4_macros.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Helper to combine version and revision into a 32-bit ID */
#define HW_INFO_REV(v, r) (((uint32_t)(v) << 16) | (uint32_t)(r))

/* Define ESP32-P4 Hardware ID (version 1, revision 0 / 3) */
#define ESP32P4_V100      HW_INFO_REV(1, 0)
#define ESP32P4_DEFAULT   HW_INFO_REV(0, 0)

typedef struct {
	uint32_t                hw_ver_rev; /* the version and revision */
	const px4_hw_mft_item_t *mft;       /* pointer to manifest array */
	uint32_t                entries;    /* number of items in array */
} px4_hw_mft_list_entry_t;

typedef px4_hw_mft_list_entry_t *px4_hw_mft_list_entry;
#define px4_hw_mft_list_uninitialized (px4_hw_mft_list_entry) -1

static const px4_hw_mft_item_t device_unsupported = {0, 0, 0};

/*
 * List of components on this ESP32-P4 board.
 * If you add optional sensors/chips, list them here.
 */

static const px4_hw_mft_item_t hw_mft_list_esp32p4[] = {
	{
		/* PX4_MFT_PX4IO = 0  — required for px4io driver */
		.present    = 1,
		.mandatory  = 0,   /* 0 = optional if IO not plugged; 1 = must work */
		.connection = px4_hw_con_onboard,
	},
	{
		/* PX4_MFT_USB = 1 */
		.present    = 1,
		.mandatory  = 1,
		.connection = px4_hw_con_onboard,
	},
	/* add more only if your enum has more IDs and you care about them */
};


static px4_hw_mft_list_entry_t mft_lists[] = {
	{ESP32P4_V100,    hw_mft_list_esp32p4, arraySize(hw_mft_list_esp32p4)},
	{ESP32P4_DEFAULT, hw_mft_list_esp32p4, arraySize(hw_mft_list_esp32p4)},
};

/************************************************************************************
 * Name: board_query_manifest
 ************************************************************************************/

#undef board_query_manifest

__EXPORT px4_hw_mft_item board_query_manifest(px4_hw_mft_item_id_t id)
{
	static px4_hw_mft_list_entry boards_manifest = px4_hw_mft_list_uninitialized;

	if (boards_manifest == px4_hw_mft_list_uninitialized) {
		uint32_t ver_rev = ((uint32_t)board_get_hw_version() << 16) | (uint32_t)board_get_hw_revision();

		for (unsigned i = 0; i < arraySize(mft_lists); i++) {
			if (mft_lists[i].hw_ver_rev == ver_rev) {
				boards_manifest = &mft_lists[i];
				break;
			}
		}

		if (boards_manifest == px4_hw_mft_list_uninitialized) {
			boards_manifest = &mft_lists[0];
			syslog(LOG_ERR, "[boot] Board %08"  PRIx32 " is not supported!\n", ver_rev);
		}
	}

	px4_hw_mft_item rv = &device_unsupported;

	if (boards_manifest != px4_hw_mft_list_uninitialized &&
	    id < boards_manifest->entries) {
		rv = &boards_manifest->mft[id];
	}

	return rv;
}
