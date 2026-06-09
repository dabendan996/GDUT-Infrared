/* USER CODE BEGIN 1 */
#include "Module_Ired.h"
#include "Module_SerialProtocol.h"
uint16_t step1;
uint16_t step2;
uint16_t step3;
uint16_t step4;
uint16_t step5;
IRManager::IRManager(uint8_t Send_Data_Type) 
{
	  this->Send_Data_Type=Send_Data_Type;
    memset(Send_Data, 0, sizeof(Send_Data));
    memset(Respon_Data, 0, sizeof(Respon_Data));
    memset(uart2_data_rec, 0, sizeof(uart2_data_rec));
    memset(uart2_data_snd, 0, sizeof(uart2_data_snd));
    uart2_data_snd[0] = 0xfc;
    uart2_data_snd[1] = 0xfb;
    uart2_data_snd[6] = 0xfd;
    uart2_data_snd[7] = 0xfe;
    memset(Data1, 0, sizeof(Data1));
    memset(Data2, 0, sizeof(Data2));
    memset(Data3, 0, sizeof(Data3));
    ir_complete_processed = 0;
    memset(Data_T4_ch1, 0, sizeof(Data_T4_ch1));
    memset(Data_T4_ch2, 0, sizeof(Data_T4_ch2));
    memset(Data_T4_ch3, 0, sizeof(Data_T4_ch3));
    memset(Data_T3_ch1, 0, sizeof(Data_T3_ch1));
    memset(Data_T3_ch2, 0, sizeof(Data_T3_ch2));
    memset(Data_T3_ch3, 0, sizeof(Data_T3_ch3));
    memset(Data_T1_ch1, 0, sizeof(Data_T1_ch1));
    memset(Data_T1_ch2, 0, sizeof(Data_T1_ch2));
    memset(Data_T1_ch3, 0, sizeof(Data_T1_ch3));
    memset(Data_T1_ch4, 0, sizeof(Data_T1_ch4));
    memset(Data_T8_ch1, 0, sizeof(Data_T8_ch1));
    memset(Data_T8_ch2, 0, sizeof(Data_T8_ch2));
    memset(Data_T8_ch3, 0, sizeof(Data_T8_ch3));
    memset(Data_T8_ch4, 0, sizeof(Data_T8_ch4));
// 队列初始化
    memset(send_queue, 0, sizeof(send_queue));
    queue_head = 0;
    queue_tail = 0;
    queue_count = 0;
    need_notify_ir_complete = 0;
    // 数据去重初始化
    memset(last_processed_data, 0, sizeof(last_processed_data));
    last_data_valid = 0;
    
    // 500ms窗口初始化
    last_response_end_time = 0;
    memset(last_received_command, 0, sizeof(last_received_command));
    memset(pending_command, 0, sizeof(pending_command));
    has_pending_command = 0;
    
    m_onDataReceived = nullptr;
    m_auto_response = false;		

}

void IRManager::init(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim4, TIM_HandleTypeDef* htim3,
                     TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim8)
{
    htim2_ptr = htim2;//定时器2是生成PWM波形的
    htim4_ptr = htim4;
    htim3_ptr = htim3;
    htim1_ptr = htim1;
    htim8_ptr = htim8;
    
    Period_Measure_Init(htim4_ptr, TIM_CHANNEL_1, Data_T4_ch1);
    Period_Measure_Init(htim4_ptr, TIM_CHANNEL_2, Data_T4_ch2);
    Period_Measure_Init(htim4_ptr, TIM_CHANNEL_3, Data_T4_ch3);
    Period_Measure_Init(htim3_ptr, TIM_CHANNEL_1, Data_T3_ch1);
    Period_Measure_Init(htim3_ptr, TIM_CHANNEL_2, Data_T3_ch2);
    Period_Measure_Init(htim3_ptr, TIM_CHANNEL_3, Data_T3_ch3);
    Period_Measure_Init(htim1_ptr, TIM_CHANNEL_1, Data_T1_ch1);
    Period_Measure_Init(htim1_ptr, TIM_CHANNEL_2, Data_T1_ch2);
    Period_Measure_Init(htim1_ptr, TIM_CHANNEL_3, Data_T1_ch3);
    Period_Measure_Init(htim1_ptr, TIM_CHANNEL_4, Data_T1_ch4);
    Period_Measure_Init(htim8_ptr, TIM_CHANNEL_1, Data_T8_ch1);
    Period_Measure_Init(htim8_ptr, TIM_CHANNEL_2, Data_T8_ch2);
    Period_Measure_Init(htim8_ptr, TIM_CHANNEL_3, Data_T8_ch3);
    Period_Measure_Init(htim8_ptr, TIM_CHANNEL_4, Data_T8_ch4);
   
}

void IRManager::Period_Measure_Init(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t* Data)
{
    HAL_TIM_IC_Stop_DMA(htim, Channel);
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
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_IC_Start_DMA(htim, Channel, Data, LENGTH);
}

// ========== 新增队列接口实现 ==========
bool IRManager::hasPendingData(void)
{
    return (queue_count > 0);
}

//void IRManager::processSendQueue(void)
//{
//    if (store_flag == 0 && queue_count > 0)
//    {
//        memcpy(uart2_data_snd + 2, send_queue[queue_head], 4);
//        queue_head = (queue_head + 1) % QUEUE_SIZE;
//        queue_count--;
//        store_flag = 1;
//    }
//}

// ========== 新增300ms窗口处理函数 ==========
bool IRManager::isDataSame(uint8_t* data1, uint8_t* data2)
{
    return (memcmp(data1, data2, 4) == 0);
}

void IRManager::storeToPendingAndCheck(uint8_t* data)
{
    memcpy(pending_command, data, 4);
    has_pending_command = 1;
}

void IRManager::checkAndSendPendingData(void)
{
    if (has_pending_command && store_flag == 0)
    {
        memcpy(uart2_data_snd + 2, pending_command, 4);
        store_flag = 1;
        has_pending_command = 0;
    }
}
void IRManager::handleNewData(uint8_t* new_data)
{
    uint64_t now_us = TimeStamp::getInstance().getMicroseconds();
    

    // 数据去重检查
    if (last_data_valid && memcmp(new_data, last_processed_data, 4) == 0)
    {

        // ★ 无论窗口内外，相同数据都不更新 last_processed_data 和 last_received_command
        return;  // 直接返回，不继续处理
    }

     else if (memcmp(new_data, last_processed_data, 4) != 0)
    {
        if (queue_count < QUEUE_SIZE)
        {
            memcpy(send_queue[queue_tail], new_data, 4);
            queue_tail = (queue_tail + 1) % QUEUE_SIZE;
            queue_count++;
        }
        last_capture_tick_us = TimeStamp::getInstance().getMicroseconds();
				// 保存本次数据供下次去重
//				for(uint8_t i=0;i<4;i++)
//				{
//					last_processed_data[i]=new_data[i];
//				}
				memcpy(last_processed_data, new_data, 4);
				
    }
		last_data_valid = 1;
}
void IRManager::getReceivedData(uint8_t* data_out)
{
    memcpy(data_out, Send_Data, 3);
}

void IRManager::setResponseData(uint8_t* data)
{
    memcpy(Respon_Data, data, 3);
}

void IRManager::setSendData(uint8_t* data)
{
    memcpy(Send_Data, data, 3);
}

Data_State IRManager::getState(void) const
{
    return state;
}

volatile uint8_t IRManager::getStoreFlag(void)
{
    return store_flag;
}

uint8_t* IRManager::getUart2DataSnd(void)
{
    return uart2_data_snd;
}

void IRManager::resetSendCounter(void)
{
    send_cnt = 0;
    Start_Send_delay = 0;
}

void IRManager::setAutoResponse(bool enable)
{
    m_auto_response = enable;
}

void IRManager::Red_Ray_DATA_Procss(uint32_t* Data, HAL_TIM_ActiveChannel channel)
{  
    uint16_t break_step = 0;
    uint16_t start_flag = 0;
    uint16_t count_data = 0;
    if (Ired_Type_Get()==R1)
		{
			uint16_t R1_Rec_start = 0;
			for(uint16_t i = 0; i < 69; i++)
			{
					if (Data[i+1] >= Data[i])
					{
							temp = Data[i+1] - Data[i];
					}
					else 
					{
							temp = Data[i+1] + (0xFFFF - Data[i]) + 1;
					}
					if(temp > 9000)
					{
							start_flag = 1;
					}
					if(start_flag == 1)
					{
							Data3[count_data] = temp;
							if((temp > 2000 && temp < 2500))
							{
									if(R1_Rec_start == 1)
									{
											Data1[count] = 1;
											count++;
									}
									if(count == 0)
									{
											R1_Rec_start = 1;
									}
							}
							else if(temp > 800 && temp < 1500)
							{
									if(count == 0 && R1_Rec_start == 0)
									{
											break_step = 1;
											break;
									}
									else if(R1_Rec_start == 1) 
									{
											Data1[count] = 0;
											count++;
									}
							}
							count_data++;
					}
			}
    }
		else if(Ired_Type_Get()==R2)
		{
				uint16_t R2_Rec_start=0;
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
										if(count==0&&R2_Rec_start==0)
										{
											break_step=1;
											break;
										}
										else if(R2_Rec_start==1)
										{
										Data1[count]=1;
										count++;
										}
								}
								else if(temp>800&&temp<1500)
								{
										if(R2_Rec_start==1)
										{
											Data1[count]=0;
											count++;
										}
										if(count==0)
										{
											R2_Rec_start=1;
										}
								}
								count_data++;
						}		
				}
		}
    if(break_step == 0)
    {
			uint8_t frame_processed = 0;  // 添加标志
        for(uint8_t i = 0; i < LENGTH/8; i++)
        {
            uint8_t val = 0;
            val |= Data1[8*i];
            val |= Data1[8*i+1] << 1;
            val |= Data1[8*i+2] << 2;
            val |= Data1[8*i+3] << 3;
            val |= Data1[8*i+4] << 4;
            val |= Data1[8*i+5] << 5;
            val |= Data1[8*i+6] << 6;
            val |= Data1[8*i+7] << 7;
            Data2[i] = val;
            if(((i+1)%4)==0)
            {
                uint8_t crc = ~(((Data2[i-1]+Data2[i-2]+Data2[i-3])*(Data2[i-1]+Data2[i-2]+Data2[i-3]))&0xFF);
                crc = crc & 0x3F;
							  Data2[i]=Data2[i]&0x7F;//有效的是7位，6位拿来校验，还有一位是标志位
							if(crc == (Data2[i]&0x3F))
							{
								step2++;
							}
//                Data2[i] = Data2[i] & 0x7F;
                if(crc == (Data2[i]&0x3F) && state == Receive)
                {
                    if(Data2[i-1]+Data2[i-2]+Data2[i-3]==0)
                    {
                        state = Receive;
                        break;
                    }
                    else
                    {
                    if (!frame_processed) {
                        uint8_t new_data[4];
                        memcpy(new_data, &Data2[i-3], 4);
                        handleNewData(new_data);
                        frame_processed = 1;
                    }             
                    }
                }
            }
        }
    }
    count = 0;
}

void IRManager::IR_SendLeader(void)
{
    HAL_TIM_PWM_Start(htim2_ptr, TIM_CHANNEL_3);
    delay_us(9000);
    HAL_TIM_PWM_Stop(htim2_ptr, TIM_CHANNEL_3);
    delay_us(4500);
}

void IRManager::IR_SendBit(uint8_t bit)
{
    if (bit)
    {
        HAL_TIM_PWM_Start(htim2_ptr, TIM_CHANNEL_3);
        delay_us(560);
        HAL_TIM_PWM_Stop(htim2_ptr, TIM_CHANNEL_3);
        delay_us(1680);
        Send_Data_cnt += 4;
    }
    else
    {
        HAL_TIM_PWM_Start(htim2_ptr, TIM_CHANNEL_3);
        delay_us(560);
        HAL_TIM_PWM_Stop(htim2_ptr, TIM_CHANNEL_3);
        delay_us(560);
        Send_Data_cnt += 2;
    }
}

void IRManager::IR_WriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        uint8_t bit = (byte >> i) & 0x01;
        IR_SendBit(bit);
    }
}

void IRManager::IR_Send(uint8_t* data, uint8_t reset_timeout)
{
    if (reset_timeout)
    {
        last_capture_tick_us = TimeStamp::getInstance().getMicroseconds();
    }
    
    uint32_t temp = 0;
    temp |= (data[0] | (data[1] << 8) | (data[2] << 16));
    temp = ((temp << 1) | Send_Data_Type);
		temp|=(Data_cnt_flag<<31);
    uint8_t crc1 = ~(((data[0]+data[1]+data[2])*(data[0]+data[1]+data[2]))&0xFF);
		crc1=crc1&0x3F;
    temp |= (crc1 << 25);
    
    IR_SendLeader();
    IR_WriteByte((uint8_t)(temp & 0xFF));
    IR_WriteByte((uint8_t)((temp >> 8) & 0xFF));
    IR_WriteByte((uint8_t)((temp >> 16) & 0xFF));
    IR_WriteByte((uint8_t)((temp >> 24) & 0xFF));
    HAL_TIM_PWM_Start(htim2_ptr, TIM_CHANNEL_3);
    delay_us(560);
    HAL_TIM_PWM_Stop(htim2_ptr, TIM_CHANNEL_3);
}

void IRManager::Disable_Capture_And_DMA(void)
{
    if (!capture_active) return;
    
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
    
    HAL_TIM_IC_Stop_DMA(htim4_ptr, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_DMA(htim4_ptr, TIM_CHANNEL_2);
    HAL_TIM_IC_Stop_DMA(htim4_ptr, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop_DMA(htim3_ptr, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_DMA(htim3_ptr, TIM_CHANNEL_2);
    HAL_TIM_IC_Stop_DMA(htim3_ptr, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop_DMA(htim1_ptr, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_DMA(htim1_ptr, TIM_CHANNEL_2);
    HAL_TIM_IC_Stop_DMA(htim1_ptr, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop_DMA(htim1_ptr, TIM_CHANNEL_4);
    HAL_TIM_IC_Stop_DMA(htim8_ptr, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_DMA(htim8_ptr, TIM_CHANNEL_2);
    HAL_TIM_IC_Stop_DMA(htim8_ptr, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop_DMA(htim8_ptr, TIM_CHANNEL_4);
    
    capture_active = 0;
}

void IRManager::Clear_Data(void)
{
	  memset(Data_T4_ch1, 0, sizeof(Data_T4_ch1));
    memset(Data_T4_ch2, 0, sizeof(Data_T4_ch2));
    memset(Data_T4_ch3, 0, sizeof(Data_T4_ch3));
    memset(Data_T3_ch1, 0, sizeof(Data_T3_ch1));
    memset(Data_T3_ch2, 0, sizeof(Data_T3_ch2));
    memset(Data_T3_ch3, 0, sizeof(Data_T3_ch3));
    memset(Data_T1_ch1, 0, sizeof(Data_T1_ch1));
    memset(Data_T1_ch2, 0, sizeof(Data_T1_ch2));
    memset(Data_T1_ch3, 0, sizeof(Data_T1_ch3));
    memset(Data_T1_ch4, 0, sizeof(Data_T1_ch4));
    memset(Data_T8_ch1, 0, sizeof(Data_T8_ch1));
    memset(Data_T8_ch2, 0, sizeof(Data_T8_ch2));
    memset(Data_T8_ch3, 0, sizeof(Data_T8_ch3));
    memset(Data_T8_ch4, 0, sizeof(Data_T8_ch4));
}
void IRManager::Enable_Capture_And_DMA(void)
{
    if (capture_active) return;
    
    memset(Data_T4_ch1, 0, sizeof(Data_T4_ch1));
    memset(Data_T4_ch2, 0, sizeof(Data_T4_ch2));
    memset(Data_T4_ch3, 0, sizeof(Data_T4_ch3));
    memset(Data_T3_ch1, 0, sizeof(Data_T3_ch1));
    memset(Data_T3_ch2, 0, sizeof(Data_T3_ch2));
    memset(Data_T3_ch3, 0, sizeof(Data_T3_ch3));
    memset(Data_T1_ch1, 0, sizeof(Data_T1_ch1));
    memset(Data_T1_ch2, 0, sizeof(Data_T1_ch2));
    memset(Data_T1_ch3, 0, sizeof(Data_T1_ch3));
    memset(Data_T1_ch4, 0, sizeof(Data_T1_ch4));
    memset(Data_T8_ch1, 0, sizeof(Data_T8_ch1));
    memset(Data_T8_ch2, 0, sizeof(Data_T8_ch2));
    memset(Data_T8_ch3, 0, sizeof(Data_T8_ch3));
    memset(Data_T8_ch4, 0, sizeof(Data_T8_ch4));
    
    HAL_TIM_IC_Start_DMA(htim4_ptr, TIM_CHANNEL_1, Data_T4_ch1, LENGTH);
    HAL_TIM_IC_Start_DMA(htim4_ptr, TIM_CHANNEL_2, Data_T4_ch2, LENGTH);
    HAL_TIM_IC_Start_DMA(htim4_ptr, TIM_CHANNEL_3, Data_T4_ch3, LENGTH);
    HAL_TIM_IC_Start_DMA(htim3_ptr, TIM_CHANNEL_1, Data_T3_ch1, LENGTH);
    HAL_TIM_IC_Start_DMA(htim3_ptr, TIM_CHANNEL_2, Data_T3_ch2, LENGTH);
    HAL_TIM_IC_Start_DMA(htim3_ptr, TIM_CHANNEL_3, Data_T3_ch3, LENGTH);
    HAL_TIM_IC_Start_DMA(htim1_ptr, TIM_CHANNEL_1, Data_T1_ch1, LENGTH);
    HAL_TIM_IC_Start_DMA(htim1_ptr, TIM_CHANNEL_2, Data_T1_ch2, LENGTH);
    HAL_TIM_IC_Start_DMA(htim1_ptr, TIM_CHANNEL_3, Data_T1_ch3, LENGTH);
    HAL_TIM_IC_Start_DMA(htim1_ptr, TIM_CHANNEL_4, Data_T1_ch4, LENGTH);
    HAL_TIM_IC_Start_DMA(htim8_ptr, TIM_CHANNEL_1, Data_T8_ch1, LENGTH);
    HAL_TIM_IC_Start_DMA(htim8_ptr, TIM_CHANNEL_2, Data_T8_ch2, LENGTH);
    HAL_TIM_IC_Start_DMA(htim8_ptr, TIM_CHANNEL_3, Data_T8_ch3, LENGTH);
    HAL_TIM_IC_Start_DMA(htim8_ptr, TIM_CHANNEL_4, Data_T8_ch4, LENGTH);
    
    __HAL_TIM_CLEAR_FLAG(htim4_ptr, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);
    __HAL_TIM_CLEAR_FLAG(htim3_ptr, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);
    __HAL_TIM_CLEAR_FLAG(htim1_ptr, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4);
    __HAL_TIM_CLEAR_FLAG(htim8_ptr, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4);
    
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
    
    capture_active = 1;
}

void IRManager::ProcessCaptureCallback(TIM_HandleTypeDef* htim)
{
	  step1++;
    if (htim->Instance == TIM4)
    {
        switch(htim->Channel)
        {
            case HAL_TIM_ACTIVE_CHANNEL_1:
							Red_Ray_DATA_Procss(Data_T4_ch1, HAL_TIM_ACTIVE_CHANNEL_1);
                HAL_TIM_IC_Start_DMA(htim4_ptr, TIM_CHANNEL_1, Data_T4_ch1, LENGTH);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_2:
                Red_Ray_DATA_Procss(Data_T4_ch2, HAL_TIM_ACTIVE_CHANNEL_2);
                HAL_TIM_IC_Start_DMA(htim4_ptr, TIM_CHANNEL_2, Data_T4_ch2, LENGTH);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_3:
                Red_Ray_DATA_Procss(Data_T4_ch3, HAL_TIM_ACTIVE_CHANNEL_3);
                HAL_TIM_IC_Start_DMA(htim4_ptr, TIM_CHANNEL_3, Data_T4_ch3, LENGTH);
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
                Red_Ray_DATA_Procss(Data_T3_ch1, HAL_TIM_ACTIVE_CHANNEL_1);
                HAL_TIM_IC_Start_DMA(htim3_ptr, TIM_CHANNEL_1, Data_T3_ch1, LENGTH);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_2:
                Red_Ray_DATA_Procss(Data_T3_ch2, HAL_TIM_ACTIVE_CHANNEL_2);
                HAL_TIM_IC_Start_DMA(htim3_ptr, TIM_CHANNEL_2, Data_T3_ch2, LENGTH);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_3:
                Red_Ray_DATA_Procss(Data_T3_ch3, HAL_TIM_ACTIVE_CHANNEL_3);
                HAL_TIM_IC_Start_DMA(htim3_ptr, TIM_CHANNEL_3, Data_T3_ch3, LENGTH);
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
                Red_Ray_DATA_Procss(Data_T1_ch1, HAL_TIM_ACTIVE_CHANNEL_1);
                HAL_TIM_IC_Start_DMA(htim1_ptr, TIM_CHANNEL_1, Data_T1_ch1, LENGTH);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_2:
                Red_Ray_DATA_Procss(Data_T1_ch2, HAL_TIM_ACTIVE_CHANNEL_2);
                HAL_TIM_IC_Start_DMA(htim1_ptr, TIM_CHANNEL_2, Data_T1_ch2, LENGTH);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_3:
                Red_Ray_DATA_Procss(Data_T1_ch3, HAL_TIM_ACTIVE_CHANNEL_3);
                HAL_TIM_IC_Start_DMA(htim1_ptr, TIM_CHANNEL_3, Data_T1_ch3, LENGTH);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_4:
                Red_Ray_DATA_Procss(Data_T1_ch4, HAL_TIM_ACTIVE_CHANNEL_4);
                HAL_TIM_IC_Start_DMA(htim1_ptr, TIM_CHANNEL_4, Data_T1_ch4, LENGTH);
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
                Red_Ray_DATA_Procss(Data_T8_ch1, HAL_TIM_ACTIVE_CHANNEL_1);
                HAL_TIM_IC_Start_DMA(htim8_ptr, TIM_CHANNEL_1, Data_T8_ch1, LENGTH);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_2:
                Red_Ray_DATA_Procss(Data_T8_ch2, HAL_TIM_ACTIVE_CHANNEL_2);
                HAL_TIM_IC_Start_DMA(htim8_ptr, TIM_CHANNEL_2, Data_T8_ch2, LENGTH);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_3:
                Red_Ray_DATA_Procss(Data_T8_ch3, HAL_TIM_ACTIVE_CHANNEL_3);
                HAL_TIM_IC_Start_DMA(htim8_ptr, TIM_CHANNEL_3, Data_T8_ch3, LENGTH);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_4:
                Red_Ray_DATA_Procss(Data_T8_ch4, HAL_TIM_ACTIVE_CHANNEL_4);
                HAL_TIM_IC_Start_DMA(htim8_ptr, TIM_CHANNEL_4, Data_T8_ch4, LENGTH);
                break;
            default:
                break;
        }
    }
}

void IRManager::HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM5)
    {
        TimeStamp::overflowCallback();
    }
}

void IRManager::process(void)
{
    uint64_t now_us = TimeStamp::getInstance().getMicroseconds();

    if(state == Send)
    {
				if (capture_active)
				{
						Disable_Capture_And_DMA();
						capture_active = 0;
				}
				Send_Data_cnt = 0;
				IR_Send(Send_Data, 1);
				send_cnt++;
				delay_us(12 * 1000);
    }
    else if(state == Receive)
    {
        if (!capture_active)
        {
            need_to_enable_capture = 1;
        }
    }
    if (need_to_enable_capture && !capture_active)
    {
        Enable_Capture_And_DMA();
        capture_active = 1;
        need_to_enable_capture = 0;
    }		
	if (state == Receive && store_flag == 0 && queue_count > 0)//防止串口数据还没发送完成造成数据覆盖，将数据存入缓存区
	{
			SerialProtocol::getInstance().pushToSendQueue(send_queue[queue_head]);
			queue_head = (queue_head + 1) % QUEUE_SIZE;
			queue_count--;
	}
}


/* USER CODE END 1 */