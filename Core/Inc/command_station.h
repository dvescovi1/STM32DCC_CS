
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h5xx_hal.h"

void CS_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void command_station_main(void);

#ifdef __cplusplus
}
#endif
