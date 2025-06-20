#include "bsp.h"
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/time.h>
#include "main.h"

// LEDs
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_4
#define LD2_GPIO_Port GPIOF
#define LD3_Pin GPIO_PIN_4
#define LD3_GPIO_Port GPIOG

// Track pins
#define TRACK_N_PIN GPIO_PIN_5
#define TRACK_P_PIN GPIO_PIN_6
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

#if 0
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
#endif
// 1. Get captured value from CCR1
uint32_t ccr = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1);

// 2. Toggle input polarity or selection (HAL doesn't support toggling active input directly)
// Use a static variable to track current polarity
static uint32_t current_input = TIM_INPUTCHANNELPOLARITY_RISING;

HAL_TIM_IC_Stop(&htim15, TIM_CHANNEL_1);

TIM_IC_InitTypeDef sConfigIC = {0};
sConfigIC.ICPolarity  = (current_input == TIM_INPUTCHANNELPOLARITY_RISING)
                        ? TIM_INPUTCHANNELPOLARITY_FALLING
                        : TIM_INPUTCHANNELPOLARITY_RISING;
sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;  // or INDIRECTTI if needed
sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
sConfigIC.ICFilter    = 0;

HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1);
HAL_TIM_IC_Start(&htim15, TIM_CHANNEL_1);

// Update current_input for next call
current_input = sConfigIC.ICPolarity;

// 3. Subtract captured value from counter
__disable_irq();
__HAL_TIM_SET_COUNTER(&htim15, __HAL_TIM_GET_COUNTER(&htim15) - ccr);
__enable_irq();

// 4. Clear capture flag (HAL clears it automatically in IRQ handler)
// If needed manually:
__HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_CC1);

return ccr;


}

// Handle timer interrupt for command station
//
// Reload ARR register of TIM15 and clear update interrupt flag.
void bsp_command_station_irq(uint32_t arr) {
#if 0
  // Reload ARR register
  LL_TIM_SetAutoReload(TIM15, arr);

  // Clear update interrupt flag
  while (LL_TIM_IsActiveFlag_UPDATE(TIM15)) LL_TIM_ClearFlag_UPDATE(TIM15);
#endif
// 1. Reload ARR register
__HAL_TIM_SET_AUTORELOAD(&htim15, arr);

// 2. Clear update interrupt flag (if needed manually)
while (__HAL_TIM_GET_FLAG(&htim15, TIM_FLAG_UPDATE))
{
    __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
}

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