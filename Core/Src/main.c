/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "shell_control.h"
#include "stdio.h"
#include "UsartScreen.h"
#include "sfud.h"
#include "easyflash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define SAMPLING
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//const double a = 0.03095;
//const double b = 3.532E-06;
//const double c = 8.614E-08;
//const double d = 1.092E-05;

//const double a = 0.004908;
//const double b = 4.862e-06;
//const double c = 2.654e-16;
//const double d = 2.157e-05;

//const double a = 23.04;
//const double b = 0.7214;
//const double c = 1.06;
//const double d = 3.98;

//const double a = 0.01431;
//const double b = 4.213e-06;
//const double c = 1.617e-15;
//const double d = 2.064e-05;


//const double p1 = 9.792e-44;
//const double p2 = -9.97e-37;
//const double p3 = 4.298e-30;
//const double p4 = -1.011e-23;
//const double p5 = 1.38e-17;
//const double p6 = -1.052e-11;
//const double p7 = 3.498e-06;
//const double p8 = 0.3071;
//const double p9 = -3.799e+05;


//const double p1 = 2.443e-26;
//const double p2 = -2.597e-19;
//const double p3 = 1.151e-12;
//const double p4 = -2.719e-06;
//const double p5 = 3.613;
//const double p6 = -2.561e+06;
//const double p7 = 7.564e+11;

//const double p1 =   3.497e-14;
//const double p2 =   -1.82e-07;
//const double p3 =      0.3159;
//const double p4 =  -1.829e+05;

//const double p1 =   4.208e-09;
//const double p2 =    -0.01458;
//const double p3 =   1.267e+04;


//const double a0 = 2.609e+04;
//const double a1 = 3.895e+04;
//const double b1 = 5990;
//const double a2 = 1.543e+04;
//const double b2 = 4880;
//const double a3 = 2529;
//const double b3 = 1253;
//const double w = 1.604e-05;


//const double a1 = 25.21;
//const double b1 = 1.826e+06;
//const double c1 = 1.058e+04;
//const double a2 = 13.57;
//const double b2 = 1.807e+06;
//const double c2 = 1.146e+04;
//const double a3 = 11.73;
//const double b3 = 1.825e+06;
//const double c3 = 4.586e+05;
//const double a4 = 2.796;
//const double b4 = 1.784e+06;
//const double c4 = 6878;
//const double a5 = 2.915;
//const double b5 = 1.772e+06;
//const double c5 = 1788;
//const double a6 = 3.975;
//const double b6 = 1.792e+06;
//const double c6 = 7139;
//const double a7 = 0.9086;
//const double b7 = 1.638e+06;
//const double c7 = 2.259e+04;
//const double a8 = 34.76;
//const double b8 = 1.878e+06;
//const double c8 = 1.693e+05;

//const double a1 = 8.726;
//const double b1 = 1.802e+06;
//const double c1 = 5756;
//const double a2 = 8.232;
//const double b2 = 1.794e+06;
//const double c2 = 1.466e+04;
//const double a3 = 0;
//const double b3 = 1.774e+06;
//const double c3 = 188.4;
//const double a4 = 36.28;
//const double b4 = 1.785e+06;
//const double c4 = 9.23e+04;

//const double a1 = 2.291e+14;
//const double b1 = 2.476e+06;
//const double c1 = 1.201e+05;
//const double a2 = 10.76;
//const double b2 = 1.803e+06;
//const double c2 = 2.768e+04;
//const double a3 = 5.187;
//const double b3 = 1.777e+06;
//const double c3 = 6.861e+04;
//const double a4 = 1.826e+13;
//const double b4 = 1.331e+07;
//const double c4 = 2.211e+06;

//const double a1 = 1.431e+14;
//const double b1 = 2.288e+06;
//const double c1 = 8.523e+04;
//const double a2 = 1.453;
//const double b2 = 1.792e+06;
//const double c2 = 1.302e+04;
//const double a3 = 2941;
//const double b3 = 2.373e+06;
//const double c3 = 2.539e+05;
//const double a4 = 1.173e+05;
//const double b4 = 5.672e+06;
//const double c4 = 1.34e+06;

//const double a1 = 1.136e+15;
//const double b1 = 3.671e+06;
//const double c1 = 3.304e+05;
//const double a2 = 1.837;
//const double b2 = 1.786e+06;
//const double c2 = 1.036e+04;
//const double a3 = 1.33;
//const double b3 = 1.762e+06;
//const double c3 = 1.206e+04;
//const double a4 = 7.864e+06;
//const double b4 = 6.625e+06;
//const double c4 = 1.373e+06;

//const double a1 = 194;
//const double b1 = 1.863e+06;
//const double c1 = 2.868e+04;
//const double a2 = 7.197;
//const double b2 = 1.798e+06;
//const double c2 = 2.484e+04;
//const double a3 = 530.1;
//const double b3 = 2.47e+06;
//const double c3 = 3.876e+05;
//const double a4 = 1.353;
//const double b4 = 1.755e+06;
//const double c4 = 1.279e+04;
//const double a5 = 14.43;
//const double b5 = 2.033e+06;
//const double c5 = 5.751e+05;

//const double a1 = 2.874;
//const double b1 = 1.754e+06;
//const double c1 = 1.488e+04;
//const double a2 = 0.4416;
//const double b2 = 1.719e+06;
//const double c2 = 9437;
//const double a3 = 3.806;
//const double b3 = 1.768e+06;
//const double c3 = 8.892e+04;
//const double a4 = 1.73e+15;
//const double b4 = 1.562e+07;
//const double c4 = 2.454e+06;

//const double a1 = 1.054e+15;
//const double b1 = 3.526e+06;
//const double c1 = 3.053e+05;
//const double a2 = 2.672;
//const double b2 = 1.788e+06;
//const double c2 = 1.047e+04;
//const double a3 = 1.389;
//const double b3 = 1.763e+06;
//const double c3 = 9646;
//const double a4 = 1.199;
//const double b4 = 1.748e+06;
//const double c4 = 9302;
//const double a5 = 2.621e+07;
//const double b5 = 7.178e+06;
//const double c5 = 1.46e+06;

const double a1 = 4.345;
const double b1 = 1.743e+06;
const double c1 = 6.402e+04;
const double a2 = -0.3796;
const double b2 = 1.708e+06;
const double c2 = 8627;
const double a3 = 1.311;
const double b3 = 1.737e+06;
const double c3 = 1.177e+04;
const double a4 = 1.597e+14;
const double b4 = 2.193e+06;
const double c4 = 7.912e+04;
const double a5 = 180.7;
const double b5 = 2.831e+06;
const double c5 = 7.44e+05;

//分段拟合31-60
//const double sa1 = 29.18;
//const double sb1 = 1.82e+06;
//const double sc1 = 9457;
//const double sa2 = 5.08;
//const double sb2 = 1.807e+06;
//const double sc2 = 5680;
//const double sa3 = 10.44;
//const double sb3 = 1.797e+06;
//const double sc3 = 1.517e+04;
//const double sa4 = -1.373;
//const double sb4 = 1.792e+06;
//const double sc4 = 3977;
//const double sa5 = 36.46;
//const double sb5 = 1.797e+06;
//const double sc5 = 1.013e+05;

const double sa1 = 39.0997499875822;
const double sa2 = 4.54969891573061;
const double sa3 = 17.4789905466391;
const double sa4 = -1.29353078245377;
const double sa5 = 33.2706648594671;
const double sb1 = 1820348.28936334;
const double sb2 = 1806697.33574431;
const double sb3 = 1799434.67168167;
const double sb4 = 1792137.08628014;
const double sb5 = 1774328.67041329;
const double sc1 = 10433.5475450764;
const double sc2 = 5537.01382155798;
const double sc3 = 18061.7881116977;
const double sc4 = 3895.38922030636;
const double sc5 = 66172.0785627557;


double paper_fit;
uint16_t paper_cnt;

uint32_t cnt_raw;
uint64_t cnt_sum;
uint32_t int_cnt;
uint8_t sample_cnt;
double sample_data[3];
uint8_t rsted = 0;

uint64_t paper[200];

uint8_t info = 0;

FATFS fs;                 // Work area (file system object) for logical drive
FIL file;                  // file objects
uint32_t byteswritten;                /* File write counts */
uint32_t bytesread;                   /* File read counts */
uint8_t filename[] = "paper.csv";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_DMA_Init(void);

static void MX_TIM2_Init(void);

static void MX_TIM6_Init(void);

static void MX_USART1_UART_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_USART3_UART_Init(void);

static void MX_SDIO_SD_Init(void);

static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
void print_paper();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    while ((USART1->SR & 0X40U) == 0);
    USART1->DR = (uint8_t) ch;
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    uint32_t max = 0;
    uint32_t min = 0xffffffff;
    uint8_t rsted = 0;
    TCHAR str_buffer[256];
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM2_Init();
    MX_TIM6_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_SDIO_SD_Init();
    MX_FATFS_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */
    shell_control_init();
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_UART_Receive_IT(&huart3, &Usart3Buffer, 1);

//    easyflash_init();
    sfud_init();
    char t[2];
    sfud_write(sfud_get_device_table(), 0, 2, "A");
    sfud_read(sfud_get_device_table(), 0, 2, t);
    logDebug("%s", t);

//    if (BSP_SD_IsDetected() == SD_PRESENT) {
//        retSD = f_mount(&fs, "", 0);
//        if (retSD) {
//            logError("mount err:%d", retSD);
//        }
//    } else {
//        logError("NO SD Card plugged!");
//    }
//
//    retSD = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
//    if (retSD) {
//        logError("file open err:%d", retSD);
//    }
//    f_printf(&file, "cnt_raw,cnt_sum,max,min,cnt_int,paper_cnt\r\n");
//    retSD = f_close(&file);
//    if (retSD) {
//        logError("file close err:%d", retSD);
//    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(500);

        SendScreenPaperNum(paper_cnt);
        if (info) {
            logInfo("cnt_raw:%ld s1:%.2lf s2:%.2lf s3:%.2lf cnt_sum:%lld max:%ld min:%ld cnt_int:%ld paper_cnt:%d\r\n",
                    cnt_raw, sample_data[0], sample_data[1], sample_data[2], cnt_sum, max, min,
                    int_cnt, ScreenCmd.CorrectNum);
//            retSD = f_open(&file, filename, FA_OPEN_APPEND | FA_WRITE | FA_READ);
//            if (retSD) {
//                logError("file open err:%d", retSD);
//            }
//            sprintf(str_buffer, "%ld,%lld,%ld,%ld,%ld,%d\r\n", cnt_raw, cnt_sum, max, min,
//                    int_cnt, ScreenCmd.CorrectNum);
//            f_printf(&file, "%s", str_buffer);
//            retSD = f_close(&file);
//            if (retSD) {
//                logError("file close err:%d", retSD);
//            }
        }

        SendScreenPaperNum(ScreenCmd.CorrectNum);
        if (ScreenCmd.Start) {
            if (rsted == 0) {
                HAL_Delay(1000);
                HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
                HAL_Delay(3000);
                sample_cnt = 0;
                int_cnt = 0;
                rsted = 1;
                info = 1;
            }
        }

        if (ScreenCmd.Finish) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
            rsted = 0;
            info = 0;
            sample_cnt = 0;
            paper[ScreenCmd.CorrectNum] = cnt_sum;
        }
        if (ScreenCmd.Stop) {
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
            rsted = 0;
            info = 0;
            sample_cnt = 0;
        }

        if (cnt_raw > 3000) {
            if (cnt_raw > max) {
                max = cnt_raw;
            }
            if (cnt_raw < min) {
                min = cnt_raw;
            }
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }

#pragma clang diagnostic pop
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
    /** Enables the Clock Security System
    */
    HAL_RCC_EnableCSS();
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

    /* USER CODE BEGIN SDIO_Init 0 */

    /* USER CODE END SDIO_Init 0 */

    /* USER CODE BEGIN SDIO_Init 1 */

    /* USER CODE END SDIO_Init 1 */
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 0;
    /* USER CODE BEGIN SDIO_Init 2 */

    /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 8400 - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 500;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 921600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    /* DMA2_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PF0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pin : PF10 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pin : SPI1_NSS_Pin */
    GPIO_InitStruct.Pin = SPI1_NSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t p_start = 0, p_end = 21;
    static uint32_t p = 0;
    static uint32_t cnt;
    static uint32_t tmp[22];
    static uint32_t tmp_next[2];
    uint32_t ex_tmp;

    cnt_raw = TIM2->CNT;
    TIM2->CNT = 0;
    cnt = cnt_raw;
    int_cnt++;
    if (int_cnt > 50 && int_cnt <= 72) {
        tmp[int_cnt - 51] = cnt;
    }

    if (int_cnt >= 73) {
        tmp_next[p++] = cnt;
    }

    if (int_cnt >= 72) {
        if (p == 2) {
            p = 0;
            tmp[0] = tmp_next[0];
            tmp[21] = tmp_next[1];
        }
        for (int i = 0; i < 21; ++i) {
            for (int j = 0; j < 21 - i; ++j) {
                if (tmp[j] > tmp[j + 1]) {
                    ex_tmp = tmp[j];
                    tmp[j] = tmp[j + 1];
                    tmp[j] = ex_tmp;
                }
            }
        }
//        logDebug("tmp:%ld",tmp[0]);
        cnt_sum = 0;
        for (int i = 1; i < 21; ++i) {
            cnt_sum += tmp[i];
//            logDebug("tmp:%ld",tmp[i]);
        }
    }
#ifndef SAMPLING
    if (int_cnt == 100 && sample_cnt < 3) {
        int_cnt = 0;
        if (cnt_sum < 1755000) {
            //0-30

//        paper_fit = a * exp(b * cnt_sum) + c * exp(d * cnt_sum);
//        paper_fit = p1 * pow(cnt_sum, 8) + p2 * pow(cnt_sum, 7) + p3 * pow(cnt_sum, 6) + p4 * pow(cnt_sum, 5) +
//                    p5 * pow(cnt_sum, 4) + p6 * pow(cnt_sum, 3) + p7 * pow(cnt_sum, 2) + p8 * pow(cnt_sum, 1) +
//                    p9 * pow(cnt_sum, 0);
//        paper_fit = p1 * pow(cnt_sum, 6) + p2 * pow(cnt_sum, 5) + p3 * pow(cnt_sum, 4) + p4 * pow(cnt_sum, 3) +
//                    p5 * pow(cnt_sum, 2) + p6 * pow(cnt_sum, 1) + p7 * pow(cnt_sum, 0);
//        paper_fit = p1 * pow(cnt_sum, 3) + p2 * pow(cnt_sum, 2) + p3 * pow(cnt_sum, 1) + p4 * pow(cnt_sum, 0);
//        paper_fit = p1 * pow(cnt_sum, 2) + p2 * pow(cnt_sum, 1) + p3 * pow(cnt_sum, 0);
            // 4阶高斯
//        paper_fit = a1 * exp(-pow(((cnt_sum - b1) / c1), 2)) + a2 * exp(-pow(((cnt_sum - b2) / c2), 2)) +
//                    a3 * exp(-pow(((cnt_sum - b3) / c3), 2)) + a4 * exp(-pow(((cnt_sum - b4) / c4), 2));
            // 5阶高斯
            paper_fit = a1 * exp(-pow(((cnt_sum - b1) / c1), 2)) + a2 * exp(-pow(((cnt_sum - b2) / c2), 2)) +
                        a3 * exp(-pow(((cnt_sum - b3) / c3), 2)) + a4 * exp(-pow(((cnt_sum - b4) / c4), 2)) +
                        a5 * exp(-pow(((cnt_sum - b5) / c5), 2));
            // 8阶高斯
//        paper_fit = a1 * exp(-pow(((cnt_sum - b1) / c1), 2)) + a2 * exp(-pow(((cnt_sum - b2) / c2), 2)) +
//                    a3 * exp(-pow(((cnt_sum - b3) / c3), 2)) + a4 * exp(-pow(((cnt_sum - b4) / c4), 2)) +
//                    a5 * exp(-pow(((cnt_sum - b5) / c5), 2)) + a6 * exp(-pow(((cnt_sum - b6) / c6), 2)) +
//                    a7 * exp(-pow(((cnt_sum - b7) / c7), 2)) + a8 * exp(-pow(((cnt_sum - b8) / c8), 2));
//        paper_fit = a0 + a1 * cos(cnt_sum * w) + b1 * sin(cnt_sum * w) + a2 * cos(cnt_sum * w) + b2 * sin(cnt_sum * w) +
//                    a3 * cos(cnt_sum * w) + b3 * sin(cnt_sum * w) + a3 * cos(cnt_sum * w) + b3 * sin(cnt_sum * w);
        } else {
            //31-60

            // 5阶高斯
            paper_fit = sa1 * exp(-pow(((cnt_sum - sb1) / sc1), 2)) + sa2 * exp(-pow(((cnt_sum - sb2) / sc2), 2)) +
                        sa3 * exp(-pow(((cnt_sum - sb3) / sc3), 2)) + sa4 * exp(-pow(((cnt_sum - sb4) / sc4), 2)) +
                        sa5 * exp(-pow(((cnt_sum - sb5) / sc5), 2));
        }
        sample_data[sample_cnt++] = paper_fit;
    }

    if (sample_cnt == 3) {
        logDebug("%.2lf %.2lf %.2lf", sample_data[0], sample_data[1], sample_data[2]);
        if (round(sample_data[0]) == round(sample_data[1]) &&
            round(sample_data[0]) == round(sample_data[2]) &&
            round(sample_data[1]) == round(sample_data[2])) {
            rsted = 0;
            ScreenCmd.Start = 0;
            sample_cnt = 0;
            paper_cnt = (uint16_t) round(sample_data[0]);
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
            info = 0;
        } else {
            sample_cnt = 0;
            paper_cnt = 0;
            rsted = 0;
            logDebug("not same");
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
        }
    }
#else
    if (int_cnt == 200) {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
    }
#endif
//    cnt_raw = TIM2->CNT;
//    TIM2->CNT = 0;
//    cnt = cnt_raw;
//
//    avg_tmp[p_start++] = cnt;
//    if (p_start == p_end) {
//        p_end++;
//        p_end %= 22;
//        p_start = p_end + 1;
//        avg_cnt++;
//    }
//    p_start %= 21;
//
//    memcpy(tmp,avg_tmp,sizeof(tmp));
//    for (int i = 0; i < 21; ++i) {
//        for (int j = 0; j < 21 - i; ++j) {
//            if(tmp[j] > tmp[j+1]){
//                ex_tmp = tmp[j];
//                tmp[j+1] = tmp[j];
//                tmp[j] = ex_tmp;
//            }
//        }
//    }
//
//    cnt_mid = tmp[10];
}

void print_paper()
{
    info = 0;

    for (int i = 0; i < 200; ++i) {
        logInfo("%d %lld", i, paper[i]);
    }
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), printp, print_paper, print paper);

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
      /* User can add his own implementation to report the file name and line number,
         ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
