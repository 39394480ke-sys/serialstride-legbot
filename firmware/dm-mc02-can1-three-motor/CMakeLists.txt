cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(STM32_TOOLCHAIN "" CACHE PATH "Path to the STM32 arm-none-eabi toolchain bin directory")
if(NOT STM32_TOOLCHAIN)
    file(GLOB STM32_TOOLCHAIN_CANDIDATES
        "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*.macos64_*/tools/bin")
    list(SORT STM32_TOOLCHAIN_CANDIDATES COMPARE NATURAL ORDER DESCENDING)
    list(LENGTH STM32_TOOLCHAIN_CANDIDATES STM32_TOOLCHAIN_COUNT)
    if(STM32_TOOLCHAIN_COUNT EQUAL 0)
        message(FATAL_ERROR "STM32CubeIDE toolchain not found; set -DSTM32_TOOLCHAIN=/path/to/bin")
    endif()
    list(GET STM32_TOOLCHAIN_CANDIDATES 0 STM32_TOOLCHAIN)
endif()
set(CMAKE_C_COMPILER "${STM32_TOOLCHAIN}/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "${STM32_TOOLCHAIN}/arm-none-eabi-gcc")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(dm_mc02_phase31_power_quiet C ASM)

set(TARGET dm_mc02_phase31_joint_a)
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(HAL ${ROOT}/vendor/Drivers/STM32H7xx_HAL_Driver)
set(CMSIS ${ROOT}/vendor/Drivers/CMSIS)
set(USB ${ROOT}/vendor/Middlewares/ST/STM32_USB_Device_Library)

add_executable(${TARGET}
    app/phase31_power_main.c
    app/dm4310_protocol.c
    app/dm4310_controller.c
    app/h6215_protocol.c
    app/motion_controller.c
    app/motion_io.c
    app/phase1_monitor.c
    app/power_quiet_controller.c
    app/feedback_timing.c
    app/usb_command_queue.c
    platform/src/board.c
    platform/src/gpio.c
    platform/src/stm32h7xx_hal_msp.c
    platform/src/stm32h7xx_it.c
    platform/src/syscalls.c
    platform/src/system_stm32h7xx.c
    platform/startup/startup_stm32h723xx.s
    platform/usb_device/App/usb_device.c
    platform/usb_device/App/usbd_cdc_if.c
    platform/usb_device/App/usbd_desc.c
    platform/usb_device/Target/usbd_conf.c
    ${USB}/Core/Src/usbd_core.c
    ${USB}/Core/Src/usbd_ctlreq.c
    ${USB}/Core/Src/usbd_ioreq.c
    ${USB}/Class/CDC/Src/usbd_cdc.c
    ${HAL}/Src/stm32h7xx_hal.c
    ${HAL}/Src/stm32h7xx_hal_cortex.c
    ${HAL}/Src/stm32h7xx_hal_exti.c
    ${HAL}/Src/stm32h7xx_hal_fdcan.c
    ${HAL}/Src/stm32h7xx_hal_flash.c
    ${HAL}/Src/stm32h7xx_hal_flash_ex.c
    ${HAL}/Src/stm32h7xx_hal_gpio.c
    ${HAL}/Src/stm32h7xx_hal_hsem.c
    ${HAL}/Src/stm32h7xx_hal_pcd.c
    ${HAL}/Src/stm32h7xx_hal_pcd_ex.c
    ${HAL}/Src/stm32h7xx_hal_pwr.c
    ${HAL}/Src/stm32h7xx_hal_pwr_ex.c
    ${HAL}/Src/stm32h7xx_hal_rcc.c
    ${HAL}/Src/stm32h7xx_hal_rcc_ex.c
    ${HAL}/Src/stm32h7xx_ll_usb.c
)

target_include_directories(${TARGET} PRIVATE
    app
    platform/inc
    platform/usb_device/App
    platform/usb_device/Target
    ${HAL}/Inc
    ${HAL}/Inc/Legacy
    ${CMSIS}/Include
    ${CMSIS}/Device/ST/STM32H7xx/Include
    ${USB}/Core/Inc
    ${USB}/Class/CDC/Inc
)

target_compile_definitions(${TARGET} PRIVATE
    USE_HAL_DRIVER
    STM32H723xx
    HSE_VALUE=24000000
)

target_compile_options(${TARGET} PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
    -Werror
    -Wno-unused-parameter
    -Wno-missing-field-initializers
    -Os
)

target_link_options(${TARGET} PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    --specs=nano.specs
    --specs=nosys.specs
    -T${ROOT}/platform/linker/STM32H723VGTX_FLASH.ld
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.map
)

set_target_properties(${TARGET} PROPERTIES SUFFIX ".elf")

add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${STM32_TOOLCHAIN}/arm-none-eabi-objcopy -O binary
            $<TARGET_FILE:${TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.bin
    COMMAND ${STM32_TOOLCHAIN}/arm-none-eabi-size $<TARGET_FILE:${TARGET}>
    COMMENT "Generating binary image and reporting memory use"
)
