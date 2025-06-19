#include "bsp.h"
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/time.h>
#include "stm32h5xx_hal.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_pwr.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_tim.h"
#include "stm32h5xx_ll_usart.h"
#include "stm32h5xx_ll_utils.h"

// LEDs
#define LD1_Pin LL_GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LD2_Pin LL_GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
#define LD3_Pin LL_GPIO_PIN_14
#define LD3_GPIO_Port GPIOB

// STLINK UART bridge
#define STLINK_RX_Pin LL_GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin LL_GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD

// Track pins
#define TRACK_N_PIN LL_GPIO_PIN_5
#define TRACK_P_PIN LL_GPIO_PIN_6
#define TRACK_GPIO_Port GPIOE
#define TRACK_N_BS_Pos GPIO_BSRR_BS5_Pos
#define TRACK_N_BR_Pos GPIO_BSRR_BR5_Pos
#define TRACK_P_BS_Pos GPIO_BSRR_BS6_Pos
#define TRACK_P_BR_Pos GPIO_BSRR_BR6_Pos

void bsp_init_decoder(void) {
  #if 0
  // Peripheral clock enable
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);

  // TIM15 interrupt
  NVIC_SetPriority(TIM15_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM15_IRQn);

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = SystemCoreClock / 1000000;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM15, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM15);
  LL_TIM_SetClockSource(TIM15, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM15, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM15);
  LL_TIM_IC_SetActiveInput(
    TIM15, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM15, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM15, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM15, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

  // Enable CH1
  LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableIT_CC1(TIM15);
  LL_TIM_EnableCounter(TIM15);
  #endif
}

void bsp_init_command_station(void) {
#if 0
  // Peripheral clock enable
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);

  // TIM15 interrupt
  NVIC_SetPriority(TIM15_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM15_IRQn);

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = SystemCoreClock / 1000000;
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
#endif
}

// Handle timer interrupt for decoder
//
// Toggle input between TI1 and TI2, subtract captured value from running
// counter and clear capture/compare interrupt flag.
uint32_t bsp_decoder_irq(void) {
  // Get captured value
  uint32_t const ccr = LL_TIM_IC_GetCaptureCH1(TIM15);

  // Toggle input TI1 and TI2
  LL_TIM_CC_DisableChannel(TIM15, LL_TIM_CHANNEL_CH1);
  LL_TIM_IC_SetActiveInput(
    TIM15,
    LL_TIM_CHANNEL_CH1,
    LL_TIM_IC_GetActiveInput(TIM15, LL_TIM_CHANNEL_CH1) ==
        LL_TIM_ACTIVEINPUT_DIRECTTI
      ? LL_TIM_ACTIVEINPUT_INDIRECTTI
      : LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);

  // Subtract captured value from running counter
  __disable_irq();
  LL_TIM_SetCounter(TIM15, LL_TIM_GetCounter(TIM15) - ccr);
  __enable_irq();

  // Clear capture/compare interrupt flag
  while (LL_TIM_IsActiveFlag_CC1(TIM15)) LL_TIM_ClearFlag_CC1(TIM15);

  return ccr;
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
  TRACK_GPIO_Port->BSRR = !N << TRACK_N_BR_Pos | !P << TRACK_P_BR_Pos |
                          N << TRACK_N_BS_Pos | P << TRACK_P_BS_Pos;
}

void bsp_write_green_led(bool on) {
  if (on) LL_GPIO_SetOutputPin(LD1_GPIO_Port, LD1_Pin);
  else LL_GPIO_ResetOutputPin(LD1_GPIO_Port, LD1_Pin);
}

void bsp_write_yellow_led(bool on) {
  if (on) LL_GPIO_SetOutputPin(LD2_GPIO_Port, LD2_Pin);
  else LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);
}

void bsp_write_red_led(bool on) {
  if (on) LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
  else LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
}

void bsp_delay(uint32_t ms) { HAL_Delay(ms); }