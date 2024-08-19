#include "uart_comp.h"

// does nothing
HAL_StatusTypeDef WRP_UART_Receive_IT(UART_HandleTypeDef * huart, uint8_t * data, uint16_t size)
{
    // remove compiler's warnings
    (void)huart;
    (void)data;
    (void)size;
    return 0;
}