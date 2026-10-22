/****************************************************************************
 * boards/espressif/esp32p4/src/esp32p4_app_desc.c
 *
 * Application descriptor structure placed at the start of DROM (.rodata_desc).
 * Required by the ESP-IDF 2nd-stage bootloader to validate and boot the image.
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

#define ESP_APP_DESC_MAGIC_WORD 0xABCD5432

typedef struct {
    uint32_t magic_word;        /* Magic word ESP_APP_DESC_MAGIC_WORD */
    uint32_t secure_version;    /* Secure version */
    uint32_t reserv1[2];        /* Reserved */
    char version[32];           /* Application version */
    char project_name[32];      /* Project name */
    char time[16];              /* Compile time */
    char date[16];              /* Compile date */
    char idf_ver[32];           /* Version of ESP-IDF */
    uint8_t app_elf_sha256[32]; /* SHA256 of the binary */
    uint16_t min_efuse_blk_rev_full; /* Min eFuse block revision */
    uint16_t max_efuse_blk_rev_full; /* Max eFuse block revision */
    uint8_t mmu_page_size;      /* MMU page size (64KB = 16) */
    uint8_t reserv2[3];         /* Reserved */
    uint32_t reserv3[4];        /* Reserved */
} esp_app_desc_t;

/* Must be placed in .rodata_desc as the very first structure in DROM */
const __attribute__((section(".rodata_desc"), used)) esp_app_desc_t esp_app_desc = {
    .magic_word             = ESP_APP_DESC_MAGIC_WORD,
    .secure_version         = 0,
    .version                = "1.18.0",
    .project_name           = "PX4_Autopilot",
    .time                   = __TIME__,
    .date                   = __DATE__,
    .idf_ver                = "v5.5",
    .min_efuse_blk_rev_full = 0,    /* v0.0 - chip revision v0.3 is supported */
    .max_efuse_blk_rev_full = 199,  /* Up to v1.99 */
    .mmu_page_size          = 16,   /* 64KB MMU page size */
};
