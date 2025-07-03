cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	TX_INCLUDE_USER_DEFINE_FILE 
	TX_SINGLE_MODE_NON_SECURE=1 
	USE_HAL_DRIVER 
	STM32H563xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/AZURE_RTOS/App
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Inc
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/inc
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/CMSIS/Device/ST/STM32H5xx/Include
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/inc
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_SOURCE_DIR}/AZURE_RTOS/App/app_azure_rtos.c
    ${CMAKE_SOURCE_DIR}/Core/Src/tx_initialize_low_level.S
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_SOURCE_DIR}/Core/Src/app_threadx.c
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


set(ThreadX_Src
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/src/tx_thread_context_restore.S
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/src/tx_thread_context_save.S
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/src/tx_thread_interrupt_control.S
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/src/tx_thread_interrupt_disable.S
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/src/tx_thread_interrupt_restore.S
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/src/tx_thread_schedule.S
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/src/tx_thread_stack_build.S
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/src/tx_thread_system_return.S
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/src/tx_timer_interrupt.S
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_initialize_high_level.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_initialize_kernel_enter.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_initialize_kernel_setup.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_stack_error_handler.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_stack_error_notify.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_system_resume.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_block_allocate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_cleanup.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_initialize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_prioritize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_block_release.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_byte_allocate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_cleanup.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_initialize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_prioritize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_search.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_byte_release.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_cleanup.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_initialize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_set.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_set_notify.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_mutex_cleanup.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_mutex_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_mutex_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_mutex_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_mutex_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_mutex_initialize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_mutex_prioritize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_mutex_priority_change.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_mutex_put.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_cleanup.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_flush.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_front_send.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_initialize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_prioritize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_receive.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_send.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_queue_send_notify.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_ceiling_put.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_cleanup.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_initialize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_prioritize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_put.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_put_notify.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_entry_exit_notify.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_identify.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_initialize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_preemption_change.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_priority_change.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_relinquish.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_reset.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_resume.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_shell_entry.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_sleep.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_stack_analyze.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_suspend.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_system_preempt_check.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_system_suspend.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_terminate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_time_slice.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_time_slice_change.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_timeout.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_thread_wait_abort.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_time_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_time_set.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_block_allocate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_block_pool_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_block_pool_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_block_pool_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_block_pool_prioritize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_block_release.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_byte_allocate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_byte_pool_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_byte_pool_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_byte_pool_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_byte_pool_prioritize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_byte_release.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_set.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_set_notify.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_mutex_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_mutex_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_mutex_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_mutex_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_mutex_prioritize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_mutex_put.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_queue_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_queue_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_queue_flush.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_queue_front_send.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_queue_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_queue_prioritize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_queue_receive.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_queue_send.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_queue_send_notify.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_ceiling_put.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_prioritize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_put.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_put_notify.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_entry_exit_notify.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_preemption_change.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_priority_change.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_relinquish.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_reset.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_resume.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_suspend.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_terminate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_time_slice_change.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_thread_wait_abort.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_activate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_change.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_deactivate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_expiration_process.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_info_get.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_initialize.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_system_activate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_system_deactivate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/tx_timer_thread_entry.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_timer_activate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_timer_change.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_timer_create.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_timer_deactivate.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_timer_delete.c
    C:/Users/dvesc/STM32Cube/Repository/STM32Cube_FW_H5_V1.5.0/Middlewares/ST/threadx/common/src/txe_timer_info_get.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ThreadX	
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create ThreadX static library
add_library(ThreadX OBJECT)
target_sources(ThreadX PRIVATE ${ThreadX_Src})
target_link_libraries(ThreadX PUBLIC stm32cubemx)

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
