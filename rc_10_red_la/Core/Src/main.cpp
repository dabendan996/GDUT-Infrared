/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "BSP_TimeStamp.h"
#include "Module_Ired.h"
#include "Module_SerialProtocol.h"
//#include "Serial1Protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum RECEIVE_FLAG
{
    WAIT_HEAD_1,    // 0xfc
    WAIT_HEAD_2,    // 0xfb
    WAIT_TYPE,      // read:1, write:0
    WAIT_DATA,
    WAIT_CHECK,     // xor
    WAIT_TAIL_1,    // 0xfd
    WAIT_TAIL_2     // 0xfe
} RECEIVE_FLAG;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_HEAD_POSITION_0 0xfc
#define FRAME_HEAD_POSITION_1 0xfb
#define FRAME_TAIL_POSITION_0 0xfd
#define FRAME_TAIL_POSITION_1 0xfe
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t uart2_data_rec[30];
uint8_t uart2_data_snd[30] = {0};
uint8_t Need_Send_Data[30] = {0};
Data_State state = Receive;
IRManager g_irManager(0);  // R1首位发0，R2首位发1
uint8_t Usart_Receive_Suc_Flag = 0;
volatile uint8_t uart_tx_complete = 0;

uint8_t data[3]={0};
uint8_t parity=0;
static uint8_t last_data[4] = {0};
static uint8_t last_valid = 0;
uint8_t current_data[4] = {0};
uint8_t parse_success = 0;
// 定义标志位
volatile uint8_t g_send_complete = 0;
volatile uint8_t g_send_success = 0;
// 定义标志位
volatile uint8_t g_data_received = 0;     // 数据接收标志
volatile uint8_t g_recv_data[3];          // 接收到的数据
volatile uint8_t g_recv_parity;           // 接收到的奇偶位

// 全局串口协议实例
SerialProtocol& g_serialProto = SerialProtocol::getInstance();
//Serial1Protocol& g_serialProto1=Serial1Protocol::getInstance();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);

/* USER CODE BEGIN PFP */
void Period_Measure_Init(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *Data);
void delay_us(uint32_t us);
void DWT_Init(void);
void Callback_Fuc(uint8_t *buf, uint16_t len);
uint8_t uart_cnt = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 串口1发送结果回调
void onSerial1SendResult(uint8_t* data, uint8_t parity, uint8_t success) {
    g_send_success = success;      // 保存结果
    g_send_complete = 1;            // 设置完成标志
}

// 等待发送完成的函数（无限等待）
//void waitForSendComplete(void) {
//    while (1) {
//        Serial1Protocol::getInstance().process();  // 必须持续处理
//        if (g_send_complete) {
//            g_send_complete = 0;  // 清除标志
//            break;  // 退出循环
//        }
//        HAL_Delay(1);
//    }
//}
// 数据接收回调
void onSerial1DataReceived(uint8_t* data, uint8_t parity) {
    // 直接使用传入的 data 和 parity
    g_recv_data[0] = data[0];
    g_recv_data[1] = data[1];
    g_recv_data[2] = data[2];
    g_recv_parity = parity;
    // 设置标志位
    g_data_received = 1;
    
    // 注意：不需要手动发送应答，协议层会自动处理
}


//// 等待数据接收（无限等待）
//void waitForData(void) {
//    while (g_data_received == 0) {
//        Serial1Protocol::getInstance().process();
//        HAL_Delay(1);
//    }
//    g_data_received = 0;  // 清除标志
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();

    /* USER CODE BEGIN Init */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM4_Init();
    MX_TIM2_Init();
    MX_TIM1_Init();
    MX_TIM8_Init();
    MX_TIM3_Init();
    MX_TIM5_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    SystemCoreClockUpdate();
    DWT_Init();

    TimeStamp *time_handle = &TimeStamp::getInstance();
    time_handle->init(&htim5);
    g_irManager.init(&htim2, &htim4, &htim3, &htim1, &htim8);

    // 初始化串口协议，传入 IRManager 指针
    g_serialProto.init(&huart2, &g_irManager);

//    g_serialProto1.init(&huart2);
		  // 设置回调
//    g_serialProto1.setSendResultCallback(onSerial1SendResult);
//		// 设置回调
//    g_serialProto1.setDataReceiveCallback(onSerial1DataReceived);
//    Serial1Protocol::getInstance().setDataReceiveCallback(onSerial1DataReceived);
//    Serial1Protocol::getInstance().setSendResultCallback(onSerial1SendResult);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        // 处理红外模块
        g_irManager.process();

        // 处理串口协议
        g_serialProto.process();
//		  	g_serialProto1.process();
        /* USER CODE END WHILE */
        /* 示例：主动发送指令 */
//        static uint8_t send_flag = 0;
//        if (send_flag == 0) {
//            send_flag = 1;
//            // 延时2秒后发送第一条指令
//            HAL_Delay(2000);
//            
//            // 串口1主动发送指令
//            uint8_t cmd1[3] = {0x01, 0x02, 0x03};
//           g_serialProto1.sendCommand(cmd1);
//             // 等待发送完成（无限等待）
//            waitForSendComplete();
//						g_serialProto1.sendCommand(cmd1);
//						waitForSendComplete();
//					  g_serialProto1.sendCommand(cmd1);
//						waitForSendComplete();
//            // 串口2主动发送红外指令（通过串口1收到的数据触发）
//            // 实际上串口2的红外发送是由串口1收到的数据触发的
//        }
//         if (g_serialProto1.hasNewData()) {
//            g_serialProto1.getReceivedData(data, &parity);
//        }
				
//        if (g_data_received) {
//            g_data_received = 0;  // 清除标志
//            // 根据接收到的数据执行操作
//            if (g_recv_data[0] == 0x01 && g_recv_data[1] == 0x02 && g_recv_data[2] == 0x03) {
//            }
//					}
//         HAL_Delay(1);
        /* USER CODE BEGIN 3 */
    }
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

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 96;
    RCC_OscInitStruct.PLL.PLLP = 1;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                   | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/**
 * @brief 定时器捕获回调
 * @param htim 定时器句柄
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    g_irManager.ProcessCaptureCallback(htim);
}

/**
 * @brief 定时器周期回调
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5) {
        TimeStamp::overflowCallback();
    }
}

/**
 * @brief 串口接收回调（空闲中断 + DMA）
 * @param huart 串口号
 * @param Size 接收数据长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        g_serialProto.onUartReceive(huart->pRxBuffPtr, Size);
    }
}

/**
 * @brief 串口发送完成回调（DMA）
 * @param huart 串口号
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        g_serialProto.onUartTxComplete();
        uart_tx_complete = 1;
    }
}

/**
 * @brief DWT 初始化，用于精密延时
 */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 微秒级延时函数
 * @param us 要延时的微秒数
 */
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/* USER CODE END 4 */

/**
  * @brief  MPU 配置
  * @param  None
  * @retval None
  */
void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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