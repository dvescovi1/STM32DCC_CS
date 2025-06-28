#include "command_station.hpp"
#include <cstdio>
#include "main.h"
#include "stm32h5xx_hal_tim.h"

extern "C" void command_station_main(void);


void CommandStation::trackOutputs(bool N, bool P) 
{ 
// TRACK_N_GPIO_Port->BSRR = (static_cast<uint32_t>(!N) << TRACK_N_BR_Pos) | (static_cast<uint32_t>(!P) << TRACK_P_BR_Pos) |
//                           (static_cast<uint32_t>(N) << TRACK_N_BS_Pos) | (static_cast<uint32_t>(P) << TRACK_P_BS_Pos);
}

void CommandStation::biDiStart() {}

void CommandStation::biDiChannel1() {}

void CommandStation::biDiChannel2() {}

void CommandStation::biDiEnd() {}

CommandStation command_station;

uint32_t arr_next{0u};

/* only use callback if NOT using custom interrupt handler! */
void CS_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
      auto const arr{command_station.transmit()};
      arr_next = arr;
      arr_next = 75 * arr_next;
      htim2.Instance->ARR = arr_next * 2;
      htim2.Instance->CCR1 = arr_next;
    }
    else if (htim->Instance == TIM15)
    {
      auto const arr{command_station.transmit()};
      arr_next = arr;
      arr_next = 75 * arr_next;
      htim15.Instance->ARR = arr_next * 2;
      htim15.Instance->CCR1 = arr_next;
    }
}

void command_station_main() {
  command_station.init({
    .num_preamble = DCC_TX_MIN_PREAMBLE_BITS,
    .bit1_duration = 58u,
    .bit0_duration = 100u,
    .flags = {.invert = false, .bidi = true},
  });
  printf("Command station: init\n");
  printf("SystemCoreClock = %lu Hz\r\n", SystemCoreClock);
  
  // Turn red LED on to indicate this board is the command station
  bsp_write_red_led(true);

  printf("\n\nBoot\n");
  
  // Enable update interrupt
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);

  HAL_Delay(200u);

#if defined(DEBUG)
  SCB->CCR &= ~SCB_CCR_UNALIGN_TRP_Msk;
#endif
  // Main loop
  // Send a few packets to test the command station
  // This is not part of the command station functionality, but rather a test
  // to see if the command station is working correctly.
  bsp_write_green_led(false);
  bsp_write_yellow_led(false);
  bsp_write_red_led(false);

dcc::Packet packet{};
  for (;;) {
    // Accelerate
    packet = dcc::make_advanced_operations_speed_packet(3u, 1u << 7u | 42u);
    command_station.packet(packet);
    printf("\nCommand station: accelerate to speed step 42\n");
    bsp_write_green_led(true);
    HAL_Delay(2000u);

    // Set function F3
    packet = dcc::make_function_group_f4_f0_packet(3u, 0b0'1000u);
    command_station.packet(packet);
    printf("Command station: set function F3\n");
    bsp_write_yellow_led(true);
    HAL_Delay(2000u);

    // Decelerate
    packet = dcc::make_advanced_operations_speed_packet(3u, 1u << 7u | 0u);
    command_station.packet(packet);
    printf("Command station: stop\n");
    bsp_write_green_led(false);
    HAL_Delay(2000u);

    // Clear function
    packet = dcc::make_function_group_f4_f0_packet(3u, 0b0'0000u);
    command_station.packet(packet);
    printf("Command station: clear function F3\n");
    bsp_write_yellow_led(false);
    HAL_Delay(2000u);
  }
}
