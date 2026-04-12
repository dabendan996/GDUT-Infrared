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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LENGTH 70
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t Data_T4_ch1[LENGTH];
uint32_t Data_T4_ch2[LENGTH];
uint32_t Data_T4_ch3[LENGTH];
uint32_t Data_T3_ch1[LENGTH];
uint32_t Data_T3_ch2[LENGTH];
uint32_t Data_T3_ch3[LENGTH];
uint32_t Data_T1_ch1[LENGTH];
uint32_t Data_T1_ch2[LENGTH];
uint32_t Data_T1_ch3[LENGTH];
uint32_t Data_T1_ch4[LENGTH];
uint32_t Data_T8_ch1[LENGTH];
uint32_t Data_T8_ch2[LENGTH];
uint32_t Data_T8_ch3[LENGTH];
uint32_t Data_T8_ch4[LENGTH];

uint8_t uaer1_data[30];

uint8_t Data1[LENGTH];
uint8_t Data2[LENGTH];
uint16_t Data3[LENGTH];
uint8_t send_flag=1;
uint8_t send_data;

uint16_t count=0;
uint16_t cp=0;
uint32_t fre;
uint32_t rec_flag;
uint16_t a=20;
 uint16_t temp=0;
static volatile uint32_t current_tick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Period_Measure_Init(TIM_HandleTypeDef *htim, uint32_t Channel,uint32_t *Data);
static void delay_us(uint32_t us);
static void DWT_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t Send[10]={11,12,13,14,17,17,16,18,71,52};
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  SystemCoreClockUpdate();
  DWT_Init(); 
	Period_Measure_Init(&htim4, TIM_CHANNEL_1,Data_T4_ch1);
  Period_Measure_Init(&htim4, TIM_CHANNEL_2,Data_T4_ch2);
  Period_Measure_Init(&htim4, TIM_CHANNEL_3,Data_T4_ch3);	
  Period_Measure_Init(&htim3, TIM_CHANNEL_1,Data_T3_ch1);
  Period_Measure_Init(&htim3, TIM_CHANNEL_2,Data_T3_ch2);
  Period_Measure_Init(&htim3, TIM_CHANNEL_3,Data_T3_ch3);		
	Period_Measure_Init(&htim1, TIM_CHANNEL_1,Data_T1_ch1);
  Period_Measure_Init(&htim1, TIM_CHANNEL_2,Data_T1_ch2);
	Period_Measure_Init(&htim1, TIM_CHANNEL_3,Data_T1_ch3);
  Period_Measure_Init(&htim1, TIM_CHANNEL_4,Data_T1_ch4);	
	Period_Measure_Init(&htim8, TIM_CHANNEL_1,Data_T8_ch1);
  Period_Measure_Init(&htim8, TIM_CHANNEL_2,Data_T8_ch2);
	Period_Measure_Init(&htim8, TIM_CHANNEL_3,Data_T8_ch3);
  Period_Measure_Init(&htim8, TIM_CHANNEL_4,Data_T8_ch4);	
//	__HAL_UART_CLEAR_IDLEFLAG(&huart1);          // 清除空闲中断标志
//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // 使能空闲中断
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,uaer1_data,30);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  IR_Send(0x44);
		HAL_Delay(a);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Red_Ray_DATA_Procss(uint32_t*Data)
{
			uint16_t start_flag=0;
			uint16_t count_data=0;
	   
        // 计算周期（考虑计数器溢出）
			for(uint16_t i=0;i<69;i++)
			{
				if (Data[i+1] >= Data[i])
        {
          temp = Data[i+1] - Data[i];
        }
        else 
        {
          // 发生了溢出
          temp = Data[i+1] + (0xFFFF - Data[i]) + 1;
        }
				if(temp>9000)
				{
					start_flag=1;
				}
					
				if(start_flag==1)
				{
				Data3[count_data]=temp;
				if((temp>2000&&temp<2500))
				{
					Data1[count]=1;
					count++;
				}
				else if(temp>800&&temp<1500)
				{
					Data1[count]=0;
					count++;
				}
				count_data++;
				}		
      }
		for(uint8_t i=0;i<(count/8);i++)
		{
			uint8_t val = 0;
			val |= Data1[8*i]   ;  // 最高位
			val |= Data1[8*i+1] << 1;
			val |= Data1[8*i+2] << 2;
			val |= Data1[8*i+3] << 3;      // 最低位
			val |= Data1[8*i+4] << 4;  // 最高位
			val |= Data1[8*i+5] << 5;
			val |= Data1[8*i+6] << 6;
			val |= Data1[8*i+7] << 7;      // 最低位
			Data2[i] = val;
			if(((i+1)%4)==0)
			{
				uint8_t crc=~(((Data2[i-1]+Data2[i-2]+Data2[i-3])*(Data2[i-1]+Data2[i-2]+Data2[i-3]))&0xFF);
				if(crc==Data2[i])
				{
					rec_flag++;
					break;
				}
			}
		}
		
		count=0;
		start_flag=0;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
			switch(htim->Channel)
			{
				case HAL_TIM_ACTIVE_CHANNEL_1:			
							Red_Ray_DATA_Procss(Data_T4_ch1);
							HAL_TIM_IC_Start_DMA(&htim4, TIM_CHANNEL_1,Data_T4_ch1,LENGTH);
							break;
				case HAL_TIM_ACTIVE_CHANNEL_2:
				      Red_Ray_DATA_Procss(Data_T4_ch2);
							HAL_TIM_IC_Start_DMA(&htim4, TIM_CHANNEL_2,Data_T4_ch2,LENGTH);
							break;
				case HAL_TIM_ACTIVE_CHANNEL_3:
				      Red_Ray_DATA_Procss(Data_T4_ch3);
							HAL_TIM_IC_Start_DMA(&htim4, TIM_CHANNEL_3,Data_T4_ch3,LENGTH);		
              break;				
				default:
        break;
			}
		}
		else if(htim->Instance == TIM3)
    {
			switch(htim->Channel)
			{
				case HAL_TIM_ACTIVE_CHANNEL_1:			
							Red_Ray_DATA_Procss(Data_T3_ch1);
							HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1,Data_T3_ch1,LENGTH);
							break;
				case HAL_TIM_ACTIVE_CHANNEL_2:
				      Red_Ray_DATA_Procss(Data_T3_ch2);
							HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_2,Data_T3_ch2,LENGTH);
							break;
				case HAL_TIM_ACTIVE_CHANNEL_3:
				      Red_Ray_DATA_Procss(Data_T3_ch3);
							HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_3,Data_T3_ch3,LENGTH);		
              break;						
				default:
        break;
			}
		}
			else if(htim->Instance == TIM1)
			{
			switch(htim->Channel)
			{
				case HAL_TIM_ACTIVE_CHANNEL_1:			
							Red_Ray_DATA_Procss(Data_T1_ch1);
							HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_1,Data_T1_ch1,LENGTH);
							break;
				case HAL_TIM_ACTIVE_CHANNEL_2:
				      Red_Ray_DATA_Procss(Data_T1_ch2);
							HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_2,Data_T1_ch2,LENGTH);
							break;
				case HAL_TIM_ACTIVE_CHANNEL_3:			
							Red_Ray_DATA_Procss(Data_T1_ch3);
							HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3,Data_T1_ch3,LENGTH);
							break;
				case HAL_TIM_ACTIVE_CHANNEL_4:
				      Red_Ray_DATA_Procss(Data_T1_ch4);
							HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_4,Data_T1_ch4,LENGTH);
							break;				
				default:
        break;
			}
		}
			else if(htim->Instance == TIM8)
			{
			switch(htim->Channel)
			{
				case HAL_TIM_ACTIVE_CHANNEL_1:			
							Red_Ray_DATA_Procss(Data_T8_ch1);
							HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_1,Data_T8_ch1,LENGTH);
							break;
				case HAL_TIM_ACTIVE_CHANNEL_2:
				      Red_Ray_DATA_Procss(Data_T8_ch2);
							HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_2,Data_T8_ch2,LENGTH);
							break;
				case HAL_TIM_ACTIVE_CHANNEL_3:			
							Red_Ray_DATA_Procss(Data_T8_ch3);
							HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_3,Data_T8_ch3,LENGTH);
							break;
				case HAL_TIM_ACTIVE_CHANNEL_4:
				      Red_Ray_DATA_Procss(Data_T8_ch4);
							HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_4,Data_T8_ch4,LENGTH);
							break;				
				default:
        break;
			}				
}
}
void Period_Measure_Init(TIM_HandleTypeDef *htim, uint32_t Channel,uint32_t *Data)
{
  // 停止定时器和中断
  HAL_TIM_IC_Stop_IT(htim, Channel);
	
  // 清除所有标志
  __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

	switch(Channel)
	{
		case TIM_CHANNEL_1:
			 __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
		   break;
		case TIM_CHANNEL_2:
			 __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2);
			 break;
		case TIM_CHANNEL_3:
			 __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC3);
		   break;
		case TIM_CHANNEL_4:
			 __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC4);
			 break;
		default:
			 break;
	}
  // 重置计数器
  __HAL_TIM_SET_COUNTER(htim, 0);
  // 初始化变量
  HAL_TIM_IC_Start_DMA(htim, Channel, 
                       Data,LENGTH);
									 
}

/**
 * @brief 微秒级延时函数（自动处理 SysTick 24 位溢出）
 * @param us 要延时的微秒数（支持大于 LENGTH ms 的值）
 * @note  依赖 SystemCoreClock 全局变量，需确保其值为 240,000,000
 */
// 初始化 DWT（在 main 开头执行一次）
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 微秒延时（基于 DWT）
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);  // SystemCoreClock = 480MHz → 480
    while ((DWT->CYCCNT - start) < ticks);
}

void IR_SendLeader(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);;       // 产生9ms的高电平; 
    delay_us(9000);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);;       // 产生4.5ms的低电平
    delay_us(4500);
}

// 发送NEC数据位0/1
void IR_SendBit(uint8_t bit) {
    if (bit) {                       //数据位1
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);;     // 产生560us的高电平
        delay_us(560);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);;     // 产生1680us的低电平
        delay_us(1680);
    } else {                         //数据位0
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);;     // 产生560us的高电平 
        delay_us(560);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);;     // 产生560us的低电平
        delay_us(560);
    }
}

// NEC数据发送一个字节,从低位开始
void IR_WriteByte(uint8_t byte) {
 
    for (uint8_t i = 0; i < 8; i++) {
        // 提取当前位的值
        uint8_t bit = (byte >> i) & 0x01;
        // 写入这一位
        IR_SendBit(bit);		
    }
}

// 发送NEC数据包_t 
void IR_Send(uint8_t data) {
			/*打开PWM发送*/
		//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	
	for(uint32_t i=0;i<100;i++)
	{
    IR_SendLeader(); 				// 发送引导码
		IR_WriteByte(0x97);			// 发送地址码
		IR_WriteByte(0x11);			// 发送地址反码
		IR_WriteByte(0x71);			// 发送命令码
	  uint8_t crc1=~(((0x97+0x11+0x71)*(0x97+0x11+0x71))&0xFF);
		IR_WriteByte(crc1);		// 发送命令反码
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// 发送结束码
		delay_us(560);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);;
		HAL_Delay(a);
	}
		for(uint32_t j=0;j<100;j++)
	{
		send_data=j;
		if (send_data==0xFF)
		{
			send_data=0;
		}
    IR_SendLeader(); 				// 发送引导码
		IR_WriteByte(0x97);			// 发送地址码
		IR_WriteByte(0x11);			// 发送地址反码
		IR_WriteByte(0x71);			// 发送命令码
    IR_WriteByte(send_data);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// 发送结束码
		delay_us(560);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);;
		HAL_Delay(a);
	}

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	
    if(huart->Instance == USART1) // 判断是否为串口1
    {
        // 这里可以添加你对接收数据的处理逻辑
        // 例如：打印数据、解析指令等
        // 示例：把收到的数据原样发回
        HAL_UART_Transmit_DMA(&huart1, uaer1_data,huart->RxXferSize - huart->RxXferCount);

        // 必须重新启动下一次接收，否则只会触发一次回调
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uaer1_data, 30);
    }
	
}
/* USER CODE END 4 */

 /* MPU Configuration */

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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
