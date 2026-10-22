/* Link fix: the ll40ls_pwm distance sensor (part of COMMON_DISTANCE_SENSOR)
 * references io_timer_channel_get_gpio_output(), which this port does not
 * provide.  That driver needs STM32-style PWM-input timers and is not used on
 * this board.  Weak, so a real implementation elsewhere would take precedence.
 */
#include <stdint.h>

__attribute__((weak)) uint32_t io_timer_channel_get_gpio_output(unsigned channel)
{
	(void)channel;
	return 0;
}
