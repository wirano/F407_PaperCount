
#ifndef  BSP_PRESENT
#define  BSP_PRESENT

/*
*********************************************************************************************************
*                                                 EXTERNS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/



#include <stm32f2xx_conf.h>
#include <rthw.h>
#include <rtthread.h>
#include "usart.h"

/*
*********************************************************************************************************
*                                                 DEFINES
*********************************************************************************************************
*/
/* board configuration */
// <o> SDCard Driver <1=>SDIO sdcard <0=>SPI MMC card
// 	<i>Default: 1
#define STM32_USE_SDIO            0

#define Period_Tim4            8000

/* whether use board external SRAM memory */
// <e>Use external SRAM memory on the board
// 	<i>Enable External SRAM memory
#define STM32_EXT_SRAM          0
//	<o>Begin Address of External SRAM
//		<i>Default: 0x68000000
#define STM32_EXT_SRAM_BEGIN    0x68000000 /* the begining address of external SRAM */
//	<o>End Address of External SRAM
//		<i>Default: 0x68080000
#define STM32_EXT_SRAM_END      0x68080000 /* the end address of external SRAM */
// </e>

// <o> Internal SRAM memory size[Kbytes] <8-128>
//	<i>Default: 128
#define STM32_SRAM_SIZE         128
#define STM32_SRAM_END          (SRAM_BASE + STM32_SRAM_SIZE * 1024)

#define USER_APP_ENTRY          (FLASH_BASE + 384 * 1024)  /* Ӧ�ó�����ڵ�ַ384Kλ�� */
#define BAK_SECTOIN_ADDR        (FLASH_BASE + 640 * 1024)  /* ��������ڵ�ַ  640Kλ�� */
//#define VECT_TAB_FLASH                                     /* ����Flash�洢������ʽ */
#define VECT_TAB_USER                                      /* �����û��Զ���������ʽ */
#define USER_VECTOR_TABLE       USER_APP_ENTRY-NVIC_VectTab_FLASH /* �û��Զ����ж�������λ�� */

/* RT_USING_UART */
#define RT_USING_UART1
//#define RT_USING_REMAP_UART1
#define RT_USING_UART2
#define RT_UART_RX_BUFFER_SIZE    256

#define RT_USING_SPI1
#define RT_USING_SPI3

//#define LED_RUN_ON              GPIO_SetBits  (GPIOA,GPIO_Pin_9)  	   //RUN
//#define LED_RUN_OFF             GPIO_ResetBits(GPIOA,GPIO_Pin_9) 	   //RUN
#define LED_RUN_ON              GPIO_SetBits  (GPIOC,GPIO_Pin_7)       //RUN
#define LED_RUN_OFF             GPIO_ResetBits(GPIOC,GPIO_Pin_7)       //RUN

/*********************************************************************************************************/
/**                                                 MACRO'S												 */
/***********************************************************************************************************/


//��Ӳ���汾�Ŷ���
#define VERSION_SOFTWARE_MAJOR        1
#define VERSION_SOFTWARE_MINOR        0
#define VERSION_HARDWARE_MAJOR        1
#define VERSION_HARDWARE_MINOR        0

/***********************************************************************************************************/
/*                                               DATA TYPES												 */
/***********************************************************************************************************/


/**********************************************************************************************************
*                                            GLOBAL VARIABLES
**********************************************************************************************************/




/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void BSP_Init(void);

void rt_hw_board_init(void);

void IWDG_Feed(void);

void rt_hw_timer_handler(void);

/*
*********************************************************************************************************
*                                           INTERRUPT SERVICES
*********************************************************************************************************
*/


#endif                                                          /* End of module include.                               */
