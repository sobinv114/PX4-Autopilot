#include <board_config.h>

#ifdef GPIO_TONE_ALARM_GPIO
#include "ToneAlarmInterfaceGPIO.cpp"
#else
#include "ToneAlarmInterfacePWM.cpp"
#endif
