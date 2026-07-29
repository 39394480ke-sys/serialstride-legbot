cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(stm32f103_boot_test C ASM)

set(TARGET_NAME stm32f103_boot_test)

add_executable(${TARGET_NAME}
    src/main.c
    startup/startup_stm32f103c8tx.s
)

target_compile_options(${TARGET_NAME} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
    -Werror
    -Os
)

target_link_options(${TARGET_NAME} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -nostdlib
    -T${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32f103c8tx.ld
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.map
)

set_target_properties(${TARGET_NAME} PROPERTIES SUFFIX ".elf")

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary
            $<TARGET_FILE:${TARGET_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.bin
    COMMAND arm-none-eabi-size $<TARGET_FILE:${TARGET_NAME}>
    COMMENT "Generating binary image and reporting memory use"
)
