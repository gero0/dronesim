cmake_minimum_required(VERSION 3.25)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_NM arm-none-eabi-nm)
set(CMAKE_STRIP arm-none-eabi-strip)
set(CMAKE_RANLIB arm-none-eabi-ranlib)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(rf_remote LANGUAGES C CXX ASM)

add_definitions(
        -DUSE_HAL_DRIVER
        -DSTM32F103xB
        -DMPU6050
        -DREMOVE_LOGGING
        -DMPL_LOG_NDEBUG=1
)

add_compile_options(
        -mcpu=cortex-m3
        -mthumb
        -Wall
        -fdata-sections
        -ffunction-sections
        -fno-exceptions
)

add_subdirectory(../Message Message)
add_subdirectory(../crc16 crc16)

add_executable(EXEC
        NRF/nRF24.c
        Core/Src/main.c
        Core/Src/queue.c
        Core/Src/lcd.c
        Core/Src/debouncer.c
        Core/Src/stm32f1xx_it.c
        Core/Src/stm32f1xx_hal_msp.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
        Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
        Core/Src/system_stm32f1xx.c
        startup_stm32f103xb.s
        Core/Src/debouncer.c Core/Inc/debouncer.h)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld)

target_link_options(EXEC PRIVATE
        -L${CMAKE_SOURCE_DIR}/MPU
        -mcpu=cortex-m3
        -mthumb
        -specs=nano.specs
        -specs=nosys.specs
        -T${LINKER_SCRIPT}
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections,--print-memory-usage
        -u _printf_float
        )

target_link_libraries(EXEC PRIVATE
        Message_Lib
        crc16_Lib
        -lc
        -lm
        -lnosys
        )


target_include_directories(EXEC PRIVATE
        Core/Inc
        USB_DEVICE/Target
        USB_DEVICE/App/
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        NRF
        Drivers/STM32F1xx_HAL_Driver/Inc
        Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F1xx/Include
        Drivers/CMSIS/Include
        madgwick/
        MPU/driver/include
        MPU/driver/eMPL
        ../DroneController/src
        ../Message/Inc
        ../crc16
        )

add_compile_options(-Os)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_definitions(-DDEBUG)
else ()
    message(STATUS "Building Debug")
    add_compile_options(-g -gdwarf-2)
endif ()

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(
        TARGET EXEC
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:EXEC> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:EXEC>
        ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")