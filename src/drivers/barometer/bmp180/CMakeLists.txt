px4_add_module(
	MODULE drivers__barometer__bmp180
	MAIN bmp180
	SRCS
		bmp180.cpp
		bmp180.h
		bmp180_math.h
		bmp180_main.cpp
	DEPENDS
		drivers_barometer
		px4_work_queue
	)
