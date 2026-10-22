############################################################################
#
#   Copyright (c) 2023-2026 PX4 Development Team. All rights reserved.
#
############################################################################


include_directories(
    ./include
    )
add_compile_options(-Wno-unused-variable)

add_library(uavcan_esp32_driver STATIC
        src/uc_esp32_can.cpp
	src/uc_esp32_clock.cpp
	src/uc_esp32_thread.cpp
    )


add_dependencies(uavcan_esp32_driver uavcan)

install(DIRECTORY include/uavcan_nuttx DESTINATION include)
install(TARGETS uavcan_esp32_driver DESTINATION lib)
