//
// Created by 或者 on 2021/6/19.
//

#include "UsartScreen.h"

ScreenCmdSt ScreenCmd=
        {
            .Start=0,
            .PaperNum=0,
            .CorrectNum=0,
            .Finish=0,
            .Stop=0,
            .Correct=0,
            .Correct_apply=0,
            .ScreenPage=0,
            .DeleteCorrect=0,
            .DeleteCorrectData=0,
            .Offset=0,
            .OffsetPaper1=0,
            .OffsetPaper2=0,
            .OffsetData=0,
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
    uint16_t DataTemp_u16=0;
    int16_t DataTemp_s16=0;

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
    else if(*(data_buffer+1)==0X05)                        //校准完成计算数据
    {
        ScreenCmd.Start=0;
        ScreenCmd.Finish=0;
        ScreenCmd.Stop=1;
        ScreenCmd.Correct=0;
        ScreenCmd.Correct_apply=1;
    }
    else if(*(data_buffer+1)==0X06)                        //切换页面
    {
        ScreenCmd.Start=0;
        ScreenCmd.Finish=0;
        ScreenCmd.Stop=1;
        ScreenCmd.Correct=0;

        if(data_buffer[2]==0x00)
        {
            ScreenCmd.ScreenPage=0;
        }
        else if(data_buffer[2]==0x01)
        {
            ScreenCmd.ScreenPage=1;
        }
    }
    else if(*(data_buffer+1)==0X07)                       //删除校准数据
    {
        ScreenCmd.DeleteCorrect=1;
        ScreenCmd.DeleteCorrectData=((data_buffer[3])<<8) | data_buffer[2];
    }
    else if(*(data_buffer+1)==0X08)                       //设置偏移——起点
    {
        ScreenCmd.OffsetPaper1=((data_buffer[3])<<8) | data_buffer[2];
    }
    else if(*(data_buffer+1)==0X09)                       //设置偏移——终点
    {
        ScreenCmd.OffsetPaper2=((data_buffer[3])<<8) | data_buffer[2];
    }
    else if(*(data_buffer+1)==0X0A)                       //设置偏移
    {
        ScreenCmd.Offset=1;
        DataTemp_u16=((data_buffer[3])<<8) | data_buffer[2];
        DataTemp_s16=(int16_t)DataTemp_u16;
        ScreenCmd.OffsetData=(float)DataTemp_s16/100;
    }
    else if(*(data_buffer+1)==0X0B)                       //删除偏移——起点
    {
        ScreenCmd.DeleteOffsetPaper1=((data_buffer[3])<<8) | data_buffer[2];
    }
    else if(*(data_buffer+1)==0X0C)                       //删除偏移——终点
    {
        ScreenCmd.DeleteOffset=1;
        ScreenCmd.DeleteOffsetPaper2=((data_buffer[3])<<8) | data_buffer[2];
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

    sprintf(StrCommand,"wav0.en=1");                        //拼接 字符串命令主干+值
    StrLen=strlen(StrCommand);                                  //计算字符串命令的长度
    StrCommand[StrLen]=0XFF;                                    //给出完整字符串命令，末尾加3个0XFF
    StrCommand[StrLen+1]=0XFF;
    StrCommand[StrLen+2]=0XFF;
    StrCommand[StrLen+3]=0X00;                                  //给字符串后加\0，防止之前的数据影响
    HAL_UART_Transmit(&huart3, StrCommand, StrLen+3, 100);//发送字符串命令给串口屏
}

//单片机向串口屏发送当前频率
void SendScreenRealFre(uint32_t f)
{
    static uint8_t StrCommand[30];
    uint8_t StrLen=0;      //字符串命令长度

    sprintf(StrCommand,"n1.val=%d",f);                          //拼接 字符串命令主干+值
    StrLen=strlen(StrCommand);                                  //计算字符串命令的长度
    StrCommand[StrLen]=0XFF;                                    //给出完整字符串命令，末尾加3个0XFF
    StrCommand[StrLen+1]=0XFF;
    StrCommand[StrLen+2]=0XFF;
    StrCommand[StrLen+3]=0X00;                                  //给字符串后加\0，防止之前的数据影响
    HAL_UART_Transmit(&huart3, StrCommand, StrLen+3, 100);//发送字符串命令给串口屏
}

//单片机向串口屏发送校准时的频率
void SendScreenCorrectFre(uint32_t f)
{
    static uint8_t StrCommand[30];
    uint8_t StrLen=0;      //字符串命令长度

    sprintf(StrCommand,"n100.val=%d",f);                        //拼接 字符串命令主干+值
    StrLen=strlen(StrCommand);                                  //计算字符串命令的长度
    StrCommand[StrLen]=0XFF;                                    //给出完整字符串命令，末尾加3个0XFF
    StrCommand[StrLen+1]=0XFF;
    StrCommand[StrLen+2]=0XFF;
    StrCommand[StrLen+3]=0X00;                                  //给字符串后加\0，防止之前的数据影响
    HAL_UART_Transmit(&huart3, StrCommand, StrLen+3, 100);//发送字符串命令给串口屏
}

//单片机向串口屏发送极板状态
void SendScreenPlateState(PlateStateEm state)
{
    static uint8_t StrCommand[30];
    uint8_t StrLen=0;      //字符串命令长度

    if(state==ShortCircuit)
    {
        sprintf(StrCommand, "t3.txt=\"Y\"");               //拼接 字符串命令主干+值
    }
    else if(state==NoShortCircuit) {
        sprintf(StrCommand, "t3.txt=\"N\"");               //拼接 字符串命令主干+值
    }

    StrLen=strlen(StrCommand);                                  //计算字符串命令的长度
    StrCommand[StrLen]=0XFF;                                    //给出完整字符串命令，末尾加3个0XFF
    StrCommand[StrLen+1]=0XFF;
    StrCommand[StrLen+2]=0XFF;
    StrCommand[StrLen+3]=0X00;                                  //给字符串后加\0，防止之前的数据影响
    HAL_UART_Transmit(&huart3, StrCommand, StrLen+3, 100);//发送字符串命令给串口屏
}
