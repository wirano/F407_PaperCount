//
// Created by 或者 on 2021/6/19.
//

#include "UsartScreen.h"

ScreenCmdSt ScreenCmd=
        {
            .Start=0,
            .PaperNum=0,
            .CorrectNum=0,
            .Finish=1,
            .Stop=0,
            .Correct=0,
        };

//接收串口屏发送到单片机的数据
void UsartScreenReceive(uint8_t data)
{
    static uint8_t DataBuffer[5];
    static uint8_t State;          //接收状态 0:接收0字节 1：接收1字节 2：接收2字节 3：接收3字节 ......

    if(State==0&&data==0XAA)       //帧头
    {
        State=1;
        DataBuffer[0]=data;
    }
    else if(State==1)              //地址帧
    {
        State=2;
        DataBuffer[1]=data;
    }
    else if(State==2)             //数据位1
    {
        State=3;
        DataBuffer[2]=data;
    }
    else if(State==3)             //数据位2
    {
        State=4;
        DataBuffer[3]=data;
    }
    else if(State==4&&data==0XBB) //帧尾
    {
        State=5;
        DataBuffer[4]=data;
        UsartScreenAnalysis(DataBuffer);
        State=0;
    }
    else
    {
        State=0;
    }
}

//解析串口屏发送到单片机的数据
void UsartScreenAnalysis(uint8_t *data_buffer)
{
    if(*(data_buffer+1)==0X01)                             //开始指令
    {
        ScreenCmd.Start=1;
        ScreenCmd.Finish=0;
        ScreenCmd.Stop=0;
        ScreenCmd.Correct=0;
//        printf("Start\r\n");
    }
    else if(*(data_buffer+1)==0X02)                        //校准指令
    {
        ScreenCmd.Start=0;
        ScreenCmd.Finish=0;
        ScreenCmd.Stop=0;
        ScreenCmd.Correct=1;
        ScreenCmd.CorrectNum = ((data_buffer[3])<<8) | data_buffer[2];
//        printf("Number=%d\r\n",ScreenCmd.CorrectNum);
    }
    else if(*(data_buffer+1)==0X03)                        //完成指令
    {
        ScreenCmd.Start=0;
        ScreenCmd.Finish=1;
        ScreenCmd.Stop=0;
        ScreenCmd.Correct=0;
//        printf("Finish\r\n");
    }
    else if(*(data_buffer+1)==0X04)                        //中止指令
    {
        ScreenCmd.Start=0;
        ScreenCmd.Finish=0;
        ScreenCmd.Stop=1;
        ScreenCmd.Correct=0;
    }
//    SendScreenPaperNum(ScreenCmd.Stop);
}

//单片机向串口屏发送纸张数
void SendScreenPaperNum(uint16_t num)
{
    static uint8_t StrCommand[30];
    uint8_t StrLen=0;      //字符串命令长度

    sprintf(StrCommand,"n0.val=%d",num);                        //拼接 字符串命令主干+值
    StrLen=strlen(StrCommand);                                  //计算字符串命令的长度
    StrCommand[StrLen]=0XFF;                                    //给出完整字符串命令，末尾加3个0XFF
    StrCommand[StrLen+1]=0XFF;
    StrCommand[StrLen+2]=0XFF;
    StrCommand[StrLen+3]=0X00;                                  //给字符串后加\0，防止之前的数据影响
    HAL_UART_Transmit(&huart3, StrCommand, StrLen+3, 100);//发送字符串命令给串口屏
}
