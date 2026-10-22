#include <drivers/drv_tone_alarm.h>
#include <px4_platform_common/defines.h>
#include <board_config.h>

namespace ToneAlarmInterface
{

void init()
{
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
}

hrt_abstime start_note(unsigned frequency)
{
	px4_arch_gpiowrite(GPIO_TONE_ALARM_GPIO, 1);
	return hrt_absolute_time();
}

void stop_note()
{
	px4_arch_gpiowrite(GPIO_TONE_ALARM_GPIO, 0);
}

} /* namespace ToneAlarmInterface */
