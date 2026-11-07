#include <nuttx/config.h>
#include <board_config.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/board_common.h>

static int _hw_version = 1;
static int _hw_revision = 0;
static const char *_hw_type_name = "ESP32P4";

int board_determine_hw_info(void)
{
	_hw_version = 1;
	_hw_revision = 0;
	_hw_type_name = "ESP32P4";
	return 0;
}

int board_get_hw_revision(void)
{
	return _hw_revision;
}

int board_get_hw_version(void)
{
	return _hw_version;
}

const char *board_get_hw_type_name(void)
{
	return _hw_type_name;
}

const char *board_get_hw_base_type_name(void)
{
	return "ESP32P4_EV";
}

__EXPORT hw_base_id_t board_get_hw_base_id(void)
{
	return HW_BASE_ID(0x300);
}
