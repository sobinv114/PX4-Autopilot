############################################################################
px4_add_module(
	MODULE drivers__pwm_input_esp32
	MAIN pwm_input
	SRCS
		pwm_input_esp32.cpp
	DEPENDS
		px4_work_queue
)
