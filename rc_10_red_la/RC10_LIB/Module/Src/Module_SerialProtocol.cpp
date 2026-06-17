#include "Module_SerialProtocol.h"
#include "BSP_TimeStamp.h"
#include "Module_Ired.h"
uint32_t times1=0;
uint32_t times2=0;
uint32_t times3=0;
// 获取单例
SerialProtocol* SerialProtocol::getInstance() {
    static SerialProtocol instance;
    return &instance;
}

// 构造函数
SerialProtocol::SerialProtocol() {
    m_huart = nullptr;
    m_irManager = nullptr;
    m_state = SERIAL_STATE_IDLE;
    store_flag = 0;
    m_rx_ready = 0;
    m_resultCallback = nullptr;
    // 初始化去重变量
    m_last_rx_time = 0;
    m_last_rx_parity = 0;
    memset(m_last_rx_data, 0, SERIAL_DATA_LEN);
}

// 初始化
void SerialProtocol::init(UART_HandleTypeDef* huart, IRManager* irManager) {
    m_huart = huart;
    m_irManager = irManager;
    m_state = SERIAL_STATE_IDLE;
    store_flag = 0;
    
    // 构建固定应答帧（数据全0，奇偶=0）
    uint8_t ack_data[SERIAL_DATA_LEN] = {0x00, 0x00, 0x00};
    
    if (m_huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_rx_buffer, 30);
//        __HAL_UART_CLEAR_IDLEFLAG(m_huart);
    }
}

// 计算校验和（6位）
uint8_t SerialProtocol::calculateChecksum(uint8_t* data) {
    uint16_t sum = data[0] + data[1] + data[2];
    uint8_t crc = ~((sum * sum) & 0xFF);
    return crc & CHECKSUM_MASK;
}



// 解析帧数据
int SerialProtocol::parseFrame(uint8_t* buffer, uint16_t size, uint8_t* data_out, uint8_t* parity_out) {
    if (size < SERIAL_FRAME_LEN) return 0;
    
    for (uint16_t i = 0; i <= size - SERIAL_FRAME_LEN; i++) {
        if (buffer[i] == SERIAL_FRAME_HEAD0 && buffer[i+1] == SERIAL_FRAME_HEAD1) {
            if (buffer[i+6] == SERIAL_FRAME_TAIL0 && buffer[i+7] == SERIAL_FRAME_TAIL1) {
                memcpy(data_out, &buffer[i+2], SERIAL_DATA_LEN);
                uint8_t check_byte = buffer[i+5];
                *parity_out = (check_byte & PARITY_BIT_MASK) >> 6;
                uint8_t received_checksum = check_byte & CHECKSUM_MASK;
                uint8_t calc_checksum = calculateChecksum(data_out);
                
                if (received_checksum == calc_checksum) {
                    return 1;
                }
            }
        }
    }
    return 0;
}


// ========== 主循环处理 ==========
void SerialProtocol::process(void) {
//	  checkAndRestartUart();
    // ========== 1. 处理串口接收数据 ==========
    if (m_rx_ready) {
        m_rx_ready = 0;
        uint8_t received_data[SERIAL_DATA_LEN];
        uint8_t received_parity;
        
        if (parseFrame(m_rx_buffer, m_rx_size, received_data, &received_parity)) {
            
            // 检查是否是串口应答（数据全0）
            uint8_t is_ack = (received_data[0] == 0x00 && 
                              received_data[1] == 0x00 && 
                              received_data[2] == 0x00);
					
            uint8_t is_stop=(received_data[0] == 0xFF && 
                              received_data[1] == 0xFF && 
                              received_data[2] == 0xFF);
					
					if(is_stop==1)
					{
						if (m_irManager)
						{
						times1++;
						m_irManager->state = Receive;
						}
					}
					else
					{

            if (m_state == SERIAL_STATE_IDLE && !is_ack) 
							{
                // 原有去重判断
                uint8_t is_same_data = (memcmp(received_data, m_last_rx_data, SERIAL_DATA_LEN) == 0);
                uint8_t is_same_parity = (received_parity == m_last_rx_parity);
                times2++;
                if (!is_same_data || (is_same_data && !is_same_parity)) {
                        // 触发红外发送
                        if (m_irManager) {
                            memcpy(m_irManager->Send_Data, received_data, SERIAL_DATA_LEN);
                            m_irManager->Data_cnt_flag = received_parity;
                            m_irManager->state = Send;
													 times3++;
                        }
                    }
                    // 更新去重记录
                    memcpy(m_last_rx_data, received_data, SERIAL_DATA_LEN);
                    m_last_rx_parity = received_parity;
                }
                // 分支C：完全相同的数据且m_ack_sent=0 → 忽略
            }				
		 }
//        memset(m_rx_buffer, 0, sizeof(m_rx_buffer));
//        m_rx_size = 0;
        if (m_huart) {
            HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_rx_buffer, 30);
            __HAL_UART_CLEAR_IDLEFLAG(m_huart);
        }
   }


}

// ========== 串口回调 ==========
void SerialProtocol::onUartReceive(uint8_t* buffer, uint16_t size) {
    if (size > 0 && size <= 30) {
        memcpy(m_rx_buffer, buffer, size);
        m_rx_size = size;
        m_rx_ready = 1;
    }
}

// 在 SerialProtocol 类中添加方法
void SerialProtocol::checkAndRestartUart(void) {
    if (!m_huart) return;
    
    // 检查串口状态
    if (__HAL_UART_GET_FLAG(m_huart, UART_FLAG_ORE) ||  // 溢出错误
        __HAL_UART_GET_FLAG(m_huart, UART_FLAG_NE) ||   // 噪声错误
        __HAL_UART_GET_FLAG(m_huart, UART_FLAG_FE) ||   // 帧错误
        __HAL_UART_GET_FLAG(m_huart, UART_FLAG_PE)) {   // 奇偶校验错误
        
        // 清除错误标志
        __HAL_UART_CLEAR_OREFLAG(m_huart);
        __HAL_UART_CLEAR_NEFLAG(m_huart);
        __HAL_UART_CLEAR_FEFLAG(m_huart);
        __HAL_UART_CLEAR_PEFLAG(m_huart);
        
        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_rx_buffer, 30);
        __HAL_UART_CLEAR_IDLEFLAG(m_huart);
    }
}
