cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32H563xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_SOURCE_DIR}/Core/Inc
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Inc
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/CMSIS/Device/ST/STM32H5xx/Include
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32h5xx_it.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32h5xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32h5xx_hal_timebase_tim.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/startup_stm32h563xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32h5xx.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_tim.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_tim_ex.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_cortex.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_icache.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc_ex.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash_ex.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_gpio.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma_ex.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr_ex.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_exti.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart_ex.c
)

# Drivers Midllewares



# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
