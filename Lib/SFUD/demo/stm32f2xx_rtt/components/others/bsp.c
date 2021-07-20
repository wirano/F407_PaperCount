/*
*********************************************************************************************************
*
*                                        BOARD SUPPORT PACKAGE
*
*                                     ST Microelectronics STM32
*                                              with the
*                                   STM3210B-EVAL Evaluation Board
*
* Filename      : bsp.c
* Version       : V1.00
* Programmer(s) : STM32F103X RT-Thread 0.3.1 USB-CDC
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#define  BSP_MODULE

#include <bsp.h>
/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/
#define DUTY_CYCLE 0
#define ADC1_DR_Address    ((u32)0x4001244C)
/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
uint16_t ADC1ConvertedValue[12];//����ADת��ֵ
/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/** This function will initial STM32 board**/
void rt_hw_board_init()
{
    BSP_Init();
    rt_components_board_init();
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
}

//******************************ʱ�����ú���***************************************
//��������: static void RCC_Configuration(void)
//�������ܣ��������й���ģ���ʱ��
//��ڲ�������
//���ڲ�������
//��    ע��Editor��Zuohao 2013-08-29    Company: BXXJS
//**********************************************************************
static void RCC_Configuration(void)
{
    RCC_ClocksTypeDef rcc_clocks;

    RCC_GetClocksFreq(&rcc_clocks);
    /* ȷ��������ȫ���� */
    RT_ASSERT(rcc_clocks.HCLK_Frequency == 120000000);

    //�����Ǹ���ģ�鿪��ʱ��
    //����GPIO
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | \
                           RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | \
                           RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG,
                           ENABLE);

    //����USART1ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    //����USART2ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    /* Enable WWDG clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
}

//********************************NVIC���ú���***************************************
//��������: void NVIC_Configuration(void)
//�������ܣ�NVIC��������
//��ڲ�������
//���ڲ�������
//��    ע��Editor��Zuohao 2013-08-29    Company: BXXJS
//**********************************************************************************
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

#if  defined(VECT_TAB_RAM)
    // Set the Vector Table base location at 0x20000000
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x00);
#elif  defined(VECT_TAB_FLASH)
    // Set the Vector Table base location at 0x08000000
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);
#elif defined(VECT_TAB_USER)
    // Set the Vector Table base location by user
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, USER_VECTOR_TABLE);
#endif

    //����NVIC���ȼ�����ΪGroup2��0-3��ռʽ���ȼ���0-3����Ӧʽ���ȼ�
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    //���ڿ��Ź��ж�����
    NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//******************************GPIO���ú���***************************************
//��������: static void GPIO_Configuration(void)
//�������ܣ���������GPIO���Ź���
//��ڲ�������
//���ڲ�������
//��    ע��Editor��Zuohao 2013-08-29    Company: BXXJS
//**********************************************************************
static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* PA13:SWDIO PA14:SWCLK for debug, can't set to output mode */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All & (~(GPIO_Pin_13 | GPIO_Pin_14));
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    /******************ϵͳ����LEDָʾ������*******************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//*******************��ʼ���������Ź�*************************************
//��������: void IWDG_Configuration(void) 
//��    ������ʼ���������Ź�
//��ڲ�������
//���ڲ�������
//��    ע����Ƶ����=4*2^prer.�����ֵֻ����256!ʱ�����(���):Tout=40K/((4*2^prer)*rlr)ֵ	 3S��ʱ
//Editor��liuqh 2013-1-16  Company: BXXJS
//*******************************************************************
static void IWDG_Configuration(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//ʹ�ܶ�IWDG->PR��IWDG->RLR��д
    IWDG_SetPrescaler(IWDG_Prescaler_64);//64��Ƶ
    IWDG_SetReload(1875);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

//*******************ι�������Ź�*************************************
//��������: void IWDG_Feed(void)
//��    ������ʼ���������Ź�
//��ڲ�������
//���ڲ�����prer:��Ƶ��:0~7(ֻ�е�3λ��Ч!)��rlr:��װ�ؼĴ���ֵ:��11λ��Ч.
//��    ע����Ƶ����=4*2^prer.�����ֵֻ����256!ʱ�����(���):Tout=40K/((4*2^prer)*rlr)ֵ
//Editor��liuqh 2013-1-16  Company: BXXJS
//*******************************************************************
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();//reload
}


/*******************************************************************************
 * Function Name  : SysTick_Configuration
 * Description    : Configures the SysTick for OS tick.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SysTick_Configuration(void)
{
    RCC_ClocksTypeDef rcc_clocks;
    rt_uint32_t cnts;

    RCC_GetClocksFreq(&rcc_clocks);

    cnts = (rt_uint32_t) rcc_clocks.HCLK_Frequency / RT_TICK_PER_SECOND;

    SysTick_Config(cnts);
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void rt_hw_timer_handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

void assert_failed(u8 *file, u32 line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* Infinite loop */
    rt_kprintf("assert failed at %s:%d \n", file, line);
    while (1) {
    }
}
/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               BSP_Init()
*
* Description : Initialize the Board Support Package (BSP).
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) This function SHOULD be called before any other BSP function is called.
*********************************************************************************************************
*/

void BSP_Init(void)
{
    RCC_Configuration();
    NVIC_Configuration();
    SysTick_Configuration();
    GPIO_Configuration();
//	TODO  ������ԣ���ʱע�Ϳ��Ź�����ʽ����ʱ��Ҫ��
// 	IWDG_Configuration();
}

