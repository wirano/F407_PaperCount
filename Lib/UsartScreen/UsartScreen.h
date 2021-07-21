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

    uint8_t ScreenPage;       //屏幕当前界面

    uint8_t Correct;          //校准指令
    uint8_t Correct_apply;    //校准完成计算数据
    uint8_t DeleteCorrect;    //删除校准指令
    uint16_t DeleteCorrectData;//删除校准数据

    uint8_t Offset;           //设置偏移命令
    uint16_t OffsetPaper1;    //偏移——起点
    uint16_t OffsetPaper2;    //偏移——终点
    float OffsetData;         //偏移量

    uint8_t DeleteOffset;     //删除偏移指令
    uint16_t DeleteOffsetPaper1;//删除偏移——起点
    uint16_t DeleteOffsetPaper2;//删除偏移——终点
}ScreenCmdSt;

typedef enum
{
    ShortCircuit,             //短路
    NoShortCircuit,           //未短路
}PlateStateEm;

void UsartScreenReceive(uint8_t data);                          //接收串口屏发送到单片机的数据
void UsartScreenAnalysis(uint8_t *data_buffer);                 //解析串口屏发送到单片机的数据
void SendScreenPaperNum(uint16_t num);                          //单片机向串口屏发送纸张数
void SendScreenRealFre(uint32_t f);                             //单片机向串口屏发送实时频率
void SendScreenCorrectFre(uint32_t f);                          //单片机向串口屏发送校准时的频率
void SendScreenPlateState(PlateStateEm state);                  //单片机向串口屏发送极板状态

extern ScreenCmdSt ScreenCmd;

#endif //ADJUSTMOTORSPEED_USARTSCREEN_H
