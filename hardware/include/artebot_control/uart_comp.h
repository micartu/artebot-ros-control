#ifndef _UART_COMP_
#define _UART_COMP_

#ifdef __cplusplus
extern "C" {
#endif
// a substitute for uart_comp for the STM32
#include <stdint.h>
#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

HAL_StatusTypeDef WRP_UART_Receive_IT(UART_HandleTypeDef * huart, uint8_t * data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* _UART_COMP_ */