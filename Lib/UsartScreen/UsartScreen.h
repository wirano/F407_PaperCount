//
// Created by 或者 on 2021/6/19.
//

#ifndef ADJUSTMOTORSPEED_USARTSCREEN_H
#define ADJUSTMOTORSPEED_USARTSCREEN_H

#include "stm32f4xx.h"
#include "main.h"

typedef struct
{
    uint8_t Start;            //开始测量命令
    uint16_t PaperNum;        //发送给屏幕的纸张数
    uint16_t CorrectNum;      //校准的纸张数
    uint8_t Finish;           //完成测量命令
    uint8_t Stop;             //中止测量命令
    uint8_t Correct;          //校准指令
}ScreenCmdSt;

void UsartScreenReceive(uint8_t data);                          //接收串口屏发送到单片机的数据
void UsartScreenAnalysis(uint8_t *data_buffer);                 //解析串口屏发送到单片机的数据
void SendScreenPaperNum(uint16_t num);                          //接收串口屏发送到单片机的数据

extern ScreenCmdSt ScreenCmd;

#endif //ADJUSTMOTORSPEED_USARTSCREEN_H
