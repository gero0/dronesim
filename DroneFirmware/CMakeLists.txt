cmake_minimum_required(VERSION 3.18)
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

project(DroneFirmware LANGUAGES C CXX ASM)

add_definitions(
    -DUSE_HAL_DRIVER
    -DSTM32F411xE
    -DMPU6050
    -DREMOVE_LOGGING
    -DMPL_LOG_NDEBUG=1
)

add_link_options(
        -L${CMAKE_SOURCE_DIR}/MPU
        -Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
        -T
        ${CMAKE_SOURCE_DIR}/STM32F411CEUx_FLASH.ld
        -mcpu=cortex-m4
        -mthumb
        -mthumb-interwork
        -u
        _printf_float
        -specs=nosys.specs)

# Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common
        -fmessage-length=0)

add_subdirectory(../DroneController DroneController)
add_subdirectory(../Message Message)
add_subdirectory(../Fusion/Fusion Fusion_dir)
add_subdirectory(../VL53L0X VL5)

add_executable(
        EXEC
        Src/main.cpp
        startup_stm32f411xe.s
        Src/stm32f4xx_it.c
        Src/stm32f4xx_hal_msp.c
        Src/system_stm32f4xx.c
        Src/mpu_interface.c
        Src/mpu_helpers.c
        Src/AHRS.cpp
        Src/vl53l0x_platform.c
        Src/vl53l0x_platform_log.c
        Src/bme_interface.c
        Src/comm_manager.cpp
        Src/freertos.c
        MPU/driver/eMPL/inv_mpu.c
        NRF/nRF24.c
        QMC/qmc5883l.c
        ../BME280_driver/bme280.c
        GYNEO/gyneo_gps_driver.c
        ../minmea/minmea.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c
        Middlewares/Third_Party/FreeRTOS/Source/croutine.c
        Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
        Middlewares/Third_Party/FreeRTOS/Source/list.c
        Middlewares/Third_Party/FreeRTOS/Source/queue.c
        Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
        Middlewares/Third_Party/FreeRTOS/Source/tasks.c
        Middlewares/Third_Party/FreeRTOS/Source/timers.c
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
        Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
        Src/BLDCController.cpp
        Src/i2c_helpers.c
)

set_property(TARGET EXEC PROPERTY CXX_STANDARD 17)
set_property(TARGET EXEC PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(EXEC PRIVATE -lc -lm -lnosys -llibmplmpu
        DroneController_Lib Message_Lib Fusion VL5_Lib)

target_include_directories(
        EXEC
        PRIVATE 
        Inc
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/CMSIS/Include
        MPU/driver/include
        MPU/driver/eMPL
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        NRF
        QMC
        GYNEO
        ../minmea
        ../DroneController/src
        ../Message/Inc
        ../Fusion/Fusion
        ../VL53L0X/core/inc
        ../BME280_driver
)

# add_compile_options(-Os)

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
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:EXEC> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
 
