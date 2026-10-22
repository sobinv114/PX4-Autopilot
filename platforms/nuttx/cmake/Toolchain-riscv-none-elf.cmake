set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR riscv)

set(TOOLCHAIN_PREFIX "riscv-none-elf-")
set(TOOLCHAIN_BIN_DIR "/home/sobin/toolchains/riscv-none-elf-gcc/bin")

# MANDATORY on ESP32 RISC-V NuttX: -mno-relax and -msmall-data-limit=0
set(ARCH_FLAGS "-march=rv32imac_zicsr_zifencei -mabi=ilp32 -mno-relax -msmall-data-limit=0")

# Safe stack & size flags (safe for ESP32-P4 early boot)
set(SAFE_OPT_FLAGS "-fomit-frame-pointer -ffunction-sections -fdata-sections")

set(CMAKE_C_FLAGS "${ARCH_FLAGS} ${SAFE_OPT_FLAGS}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${ARCH_FLAGS} ${SAFE_OPT_FLAGS} -fno-exceptions -fno-rtti" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${ARCH_FLAGS}" CACHE STRING "" FORCE)

set(CMAKE_C_COMPILER "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}g++")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}gcc")

set(CMAKE_OBJCOPY "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}objcopy")
set(CMAKE_OBJDUMP "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}objdump")
set(CMAKE_SIZE "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}size")
set(CMAKE_AR "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}gcc-ar")
set(CMAKE_RANLIB "${TOOLCHAIN_BIN_DIR}/${TOOLCHAIN_PREFIX}gcc-ranlib")

set(CROSSDEV "riscv-none-elf-" CACHE STRING "" FORCE)

set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)
