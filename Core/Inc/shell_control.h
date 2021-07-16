//
// Created by wirano on 2021/3/15.
//

#ifndef SHELL_CONTROL_H
#define SHELL_CONTROL_H

#include "main.h"
#include "log.h"


extern UART_HandleTypeDef huart1;

extern uint8_t Usart3Buffer;   //串口2接收1字节缓存

void shell_control_init();

#endif //SHELL_CONTROL_H
