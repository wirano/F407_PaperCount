//
// Created by wirano on 2021/3/15.
//

#include "shell_control.h"
#include "shell_port.h"
#include "UsartScreen.h"

Shell shell;
Log slog = {
        .write = uartLogWrite,
        .active = true,
        .level = LOG_DEBUG
};
uint8_t usart1_rec;

uint8_t Usart3Buffer;   //串口2接收1字节缓存

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
    else if(huart->Instance == huart3.Instance)
    {
        UsartScreenReceive(Usart3Buffer);
        HAL_UART_Receive_IT(&huart3,&Usart3Buffer,1);
    }

}
