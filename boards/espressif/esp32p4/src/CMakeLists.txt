add_library(drivers_board
        io_timer_stubs.c
	init.c
	can.c
	led.c
	i2c.cpp
	spi.cpp
	timer_config.cpp
	# esp32p4_board_usb.c
	manifest.c
	esp32p4_adc.c
	mtd.cpp
	esp32p4_app_desc.c
	esp32p4_board_tim.c
	board_hw_info.c
	sdmmc.c
)


target_link_libraries(drivers_board
	PRIVATE
		arch_io_pins
		arch_spi
		drivers__led # drv_led_start
		nuttx_arch # sdio
		nuttx_drivers # sdio
		px4_layer
)



# Global: board_config.h is included from many modules (version, ekf2, ...)
set(ESP32P4_HAL_INCLUDES
        ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx/arch/risc-v/src/esp32p4/esp-hal-3rdparty/components/soc/include
	${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx/arch/risc-v/src/esp32p4/esp-hal-3rdparty/components/soc/esp32p4/include
	${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx/arch/risc-v/src/esp32p4/esp-hal-3rdparty/components/esp_common/include
	${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx/arch/risc-v/src/esp32p4/esp-hal-3rdparty/nuttx/esp32p4/include
	${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx/arch/risc-v/src/esp32p4/esp-hal-3rdparty/components/hal/include
        ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx/arch/risc-v/src/esp32p4/esp-hal-3rdparty/components/esp_hal_ana_conv/include
)
include_directories(${ESP32P4_HAL_INCLUDES})
