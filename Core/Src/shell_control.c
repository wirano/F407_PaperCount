//
// Created by wirano on 2021/3/15.
//

#include "shell_control.h"
#include "shell_port.h"

Shell shell;
Log slog = {
        .write = uartLogWrite,
        .active = true,
        .level = LOG_DEBUG
};
uint8_t usart1_rec;

void shell_control_init()
{
    HAL_UART_Receive_IT(&huart1, &usart1_rec, sizeof(usart1_rec));
    shell_ported_init(&shell);
    logRegister(&slog, &shell);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart1.Instance)
    {
        shellHandler(&shell, usart1_rec);
        HAL_UART_Receive_IT(&huart1, &usart1_rec, sizeof(usart1_rec));
    }
}
