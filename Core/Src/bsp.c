#include "bsp.h"
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/time.h>
#include "main.h"
#include <sys/time.h>
#include "stm32h5xx_ll_icache.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_system.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_dma.h"
#include "stm32h5xx_ll_tim.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_bus.h"


// Track pins
#define TRACK_N_PIN GPIO_PIN_5
#define TRACK_P_PIN GPIO_PIN_6
#define TRACK_GPIO_Port GPIOE
#define TRACK_N_BS_Pos GPIO_BSRR_BS5_Pos
#define TRACK_N_BR_Pos GPIO_BSRR_BR5_Pos
#define TRACK_P_BS_Pos GPIO_BSRR_BS6_Pos
#define TRACK_P_BR_Pos GPIO_BSRR_BR6_Pos


void bsp_init_command_station(void) {
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = TRACK_N_PIN | TRACK_P_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(TRACK_GPIO_Port, &GPIO_InitStruct);

  // Peripheral clock enable
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);

  // TIM15 interrupt
  NVIC_SetPriority(TIM15_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM15_IRQn);

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = (uint16_t)(SystemCoreClock / 1000000 -1);
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM15, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM15);
  LL_TIM_SetClockSource(TIM15, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM15, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM15);

  // Enable update
  LL_TIM_EnableIT_UPDATE(TIM15);
  LL_TIM_EnableCounter(TIM15);
}


// Handle timer interrupt for command station
//
// Reload ARR register of TIM15 and clear update interrupt flag.
void bsp_command_station_irq(uint32_t arr) {
  // Reload ARR register
  LL_TIM_SetAutoReload(TIM15, arr);

  // Clear update interrupt flag
  while (LL_TIM_IsActiveFlag_UPDATE(TIM15)) LL_TIM_ClearFlag_UPDATE(TIM15);
}

void bsp_write_track(bool N, bool P) {
  TRACK_GPIO_Port->BSRR = ((uint32_t)(!N) << TRACK_N_BR_Pos) | ((uint32_t)(!P) << TRACK_P_BR_Pos) |
                          ((uint32_t)(N) << TRACK_N_BS_Pos) | ((uint32_t)(P) << TRACK_P_BS_Pos);
}

void bsp_write_green_led(bool on) {
  if (on) HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  else HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
}

void bsp_write_yellow_led(bool on) {
  if (on) HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  else HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void bsp_write_red_led(bool on) {
  if (on) HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  else HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}

void bsp_delay(uint32_t ms) { HAL_Delay(ms); }