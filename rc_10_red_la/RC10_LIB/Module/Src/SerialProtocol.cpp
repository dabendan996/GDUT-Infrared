#include "SerialProtocol.h"
#include "BSP_TimeStamp.h"
#include "Module_Ired.h"

// 获取单例
SerialProtocol& SerialProtocol::getInstance() {
    static SerialProtocol instance;
    return instance;
}

// 构造函数
SerialProtocol::SerialProtocol() {
    m_huart = nullptr;
    m_irManager = nullptr;
    m_state = SERIAL_STATE_IDLE;
    store_flag = 0;
    m_send_batch_count = 0;
    m_retry_count = 0;
    m_current_send_parity = 0;
    m_tx_complete = 1;
    m_rx_ready = 0;
    m_resultCallback = nullptr;
    m_send_start_time = 0;
    m_send_first_start_time = 0;
    m_waiting_ir_complete = 0;
    
    // 初始化去重变量
    m_last_rx_time = 0;
    m_last_rx_parity = 0;
    memset(m_last_rx_data, 0, SERIAL_DATA_LEN);
    
    m_queue_head = 0;
    m_queue_tail = 0;
    m_queue_count = 0;
    memset(m_send_queue, 0, sizeof(m_send_queue));
    memset(m_current_send_data, 0, SERIAL_DATA_LEN);
    memset(m_rx_buffer, 0, sizeof(m_rx_buffer));
    memset(m_ack_frame, 0, SERIAL_FRAME_LEN);
    // 新增初始化
    memset(m_last_processed_data, 0, SERIAL_DATA_LEN);
    m_last_processed_parity = 0;
    m_need_send_ack = 0;
    m_ack_sent = 0;
    m_ack_retry_count = 0;		
}

// 初始化
void SerialProtocol::init(UART_HandleTypeDef* huart, IRManager* irManager) {
    m_huart = huart;
    m_irManager = irManager;
    m_state = SERIAL_STATE_IDLE;
    store_flag = 0;
    m_tx_complete = 1;
    m_waiting_ir_complete = 0;
    
    // 构建固定应答帧（数据全0，奇偶=0）
    uint8_t ack_data[SERIAL_DATA_LEN] = {0x00, 0x00, 0x00};
    buildFrame(ack_data, 0, m_ack_frame);
    
    if (m_huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_rx_buffer, 30);
        __HAL_UART_CLEAR_IDLEFLAG(m_huart);
    }
}

// 获取当前时间(ms)
uint32_t SerialProtocol::getTickMs(void) {
    return TimeStamp::getInstance().getMilliseconds();
}

// 计算校验和（6位）
uint8_t SerialProtocol::calculateChecksum(uint8_t* data) {
    uint16_t sum = data[0] + data[1] + data[2];
    uint8_t crc = ~((sum * sum) & 0xFF);
    return crc & CHECKSUM_MASK;
}

// 构建完整帧
void SerialProtocol::buildFrame(uint8_t* data, uint8_t parity, uint8_t* frame_out) {
    frame_out[0] = SERIAL_FRAME_HEAD0;
    frame_out[1] = SERIAL_FRAME_HEAD1;
    memcpy(&frame_out[2], data, SERIAL_DATA_LEN);
    
    uint8_t checksum = calculateChecksum(data);
    uint8_t check_byte = checksum | ((parity & 0x01) << 6);
    
    frame_out[5] = check_byte;
    frame_out[6] = SERIAL_FRAME_TAIL0;
    frame_out[7] = SERIAL_FRAME_TAIL1;
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

// 发送单次帧
void SerialProtocol::sendFrame(uint8_t* data, uint8_t parity) {
    if (!m_huart || m_tx_complete == 0) return;
    
    m_tx_complete = 0;
    buildFrame(data, parity, m_uart_send_frame);
    HAL_UART_Transmit_DMA(m_huart, m_uart_send_frame, SERIAL_FRAME_LEN);
}

// 开始发送当前数据
void SerialProtocol::startSending(void) {
    m_send_batch_count = 0;
    m_state = SERIAL_STATE_SENDING;
}

// 通知发送结果
void SerialProtocol::notifySendResult(uint8_t success) {
    if (m_resultCallback) {
        m_resultCallback(m_current_send_data, success);
    }
}

// 设置发送结果回调
void SerialProtocol::setSendResultCallback(SerialSendResultCallback callback) {
    m_resultCallback = callback;
}

// ========== 新增：发送一次应答 ==========
void SerialProtocol::sendOneAckFrame(void) {
    if (!m_huart || m_tx_complete == 0) return;
    
    uint8_t ack_data[SERIAL_DATA_LEN] = {0x00, 0x00, 0x00};
    sendFrame(ack_data, 0);
}

// ========== 红外完成回调 ==========
void SerialProtocol::onInfraredSendComplete(uint8_t success) {
    if (m_waiting_ir_complete) {
        m_waiting_ir_complete = 0;
        
        if (success) {
            // 红外发送成功，标记需要发送应答
            m_need_send_ack = 1;
        }
    }
}

// ========== 队列操作 ==========
bool SerialProtocol::pushToSendQueue(uint8_t* data_4bytes) {
    if (m_queue_count >= SERIAL_QUEUE_SIZE) {
        return false;
    }
    
    memcpy(m_send_queue[m_queue_tail], data_4bytes, 4);
    m_queue_tail = (m_queue_tail + 1) % SERIAL_QUEUE_SIZE;
    m_queue_count++;
    
    store_flag = 1;
    return true;
}

bool SerialProtocol::popFromSendQueue(uint8_t* data_4bytes) {
    if (m_queue_count == 0) {
        return false;
    }
    
    memcpy(data_4bytes, m_send_queue[m_queue_head], 4);
    m_queue_head = (m_queue_head + 1) % SERIAL_QUEUE_SIZE;
    m_queue_count--;
    
    if (m_queue_count == 0) {
        store_flag = 0;
    }
    return true;
}

bool SerialProtocol::isSendQueueEmpty(void) {
    return (m_queue_count == 0);
}

// ========== 主循环处理 ==========
void SerialProtocol::process(void) {
    uint32_t now = getTickMs();
    
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
            
            if (m_state == SERIAL_STATE_WAITING_ACK && is_ack) {
                // 收到串口应答，发送成功
                m_state = SERIAL_STATE_IDLE;
                m_retry_count = 0;
                notifySendResult(1);
            }
            else if (m_state == SERIAL_STATE_IDLE && !is_ack) {
                
                // 判断是否与上次处理的数据相同（用于应答重发）
                uint8_t is_same_as_last_processed = (memcmp(received_data, m_last_processed_data, SERIAL_DATA_LEN) == 0 &&
                                                      received_parity == m_last_processed_parity);
                
                // 原有去重判断
                uint8_t is_same_data = (memcmp(received_data, m_last_rx_data, SERIAL_DATA_LEN) == 0);
                uint8_t is_same_parity = (received_parity == m_last_rx_parity);
                
                // ? 分支A：应答重发（串口1没收到应答，重发相同指令）
                if (is_same_as_last_processed && m_ack_sent && m_need_send_ack == 0) {
                    // 串口1没收到应答，重发一次应答
                    sendOneAckFrame();
                    m_ack_sent = 1;
                    m_ack_retry_count++;
                }
                // ? 分支B：新数据或奇偶不同的数据
                else if (!is_same_data || (is_same_data && !is_same_parity)) {
                    
                    if (!is_same_as_last_processed) {
                        // 新指令
                        memcpy(m_last_processed_data, received_data, SERIAL_DATA_LEN);
                        m_last_processed_parity = received_parity;
                        m_need_send_ack = 0;
                        m_ack_sent = 0;  // 新指令，重置应答标志
                        m_ack_retry_count = 0;
                        
                        // 触发红外发送
                        if (m_irManager) {
                            m_waiting_ir_complete = 1;
                            memcpy(m_irManager->Send_Data, received_data, SERIAL_DATA_LEN);
                            m_irManager->Data_cnt_flag = received_parity;
                            m_irManager->state = Send;
                        }
                    }
                    
                    // 更新去重记录
                    memcpy(m_last_rx_data, received_data, SERIAL_DATA_LEN);
                    m_last_rx_parity = received_parity;
                    m_last_rx_time = now;
                }
                // 分支C：完全相同的数据且m_ack_sent=0 → 忽略
            }
        }
        
        // 重新启动接收
        if (m_huart) {
            HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_rx_buffer, 30);
            __HAL_UART_CLEAR_IDLEFLAG(m_huart);
        }
    }
    
    // ========== 1.5 红外完成后发送应答 ==========
    if (m_need_send_ack && !m_ack_sent && m_state == SERIAL_STATE_IDLE) {
        m_need_send_ack = 0;
        sendOneAckFrame();
        m_ack_sent = 1;
	//		  m_waiting_ir_complete = 0;
    }
    
    // ========== 2. 处理发送队列 ==========
    if (store_flag && m_state == SERIAL_STATE_IDLE && m_queue_count > 0) {
        uint8_t queue_data[4];
        
        if (popFromSendQueue(queue_data)) {
            memcpy(m_current_send_data, queue_data, SERIAL_DATA_LEN);
            m_current_send_parity = (queue_data[3] >> 6) & 0x01;
            
            m_send_first_start_time = now;
            m_retry_count = 0;
            startSending();
        }
    }
    
    // ========== 3. 发送状态机 ==========
    switch (m_state) {
        case SERIAL_STATE_SENDING:
            if (m_tx_complete) {
                m_send_batch_count++;
                
                if (m_send_batch_count < SERIAL_SEND_TIMES) {
                    sendFrame(m_current_send_data, m_current_send_parity);
                } else {
                    m_state = SERIAL_STATE_WAITING_ACK;
                    m_send_start_time = now;
                }
            }
            break;
            
        case SERIAL_STATE_WAITING_ACK:
            if ((now - m_send_start_time) >= SERIAL_WAIT_ACK_MS) {
                if ((now - m_send_first_start_time) >= SERIAL_TOTAL_TIMEOUT_MS) {
                    m_state = SERIAL_STATE_IDLE;
                    m_retry_count = 0;
                    notifySendResult(0);
                } else {
                    m_retry_count++;
                    startSending();
                }
            }
            break;
            
        default:
            break;
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

void SerialProtocol::onUartTxComplete(void) {
    m_tx_complete = 1;
}