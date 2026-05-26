// SerialProtocol.h
#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include "main.h"
#include "string.h"

// 前向声明
class IRManager;

#ifdef __cplusplus
extern "C" {
#endif

/* 协议常量定义 */
#define SERIAL_FRAME_HEAD0     0xFC
#define SERIAL_FRAME_HEAD1     0xFB
#define SERIAL_FRAME_TAIL0     0xFD
#define SERIAL_FRAME_TAIL1     0xFE

#define SERIAL_DATA_LEN        3
#define SERIAL_FRAME_LEN       8

#define SERIAL_SEND_TIMES      2    // 每组发送3次
#define SERIAL_WAIT_ACK_MS     400   // 等待应答超时400ms
#define SERIAL_TOTAL_TIMEOUT_MS   5000   // 总超时5秒
#define SERIAL_QUEUE_SIZE      20    // 发送队列大小

/* 校验字节位定义 */
#define CHECKSUM_MASK          0x3F    // 低6位校验值掩码
#define PARITY_BIT_MASK        0x40    // bit6 奇偶标志位

/* 状态枚举 */
typedef enum {
    SERIAL_STATE_IDLE,           // 空闲
    SERIAL_STATE_SENDING,        // 发送中
    SERIAL_STATE_WAITING_ACK     // 等待应答
} SerialState_t;

/* 发送结果回调 */
typedef void (*SerialSendResultCallback)(uint8_t* data, uint8_t success);

class SerialProtocol {
public:
    // 获取单例
    static SerialProtocol& getInstance();
    
    // 初始化
    void init(UART_HandleTypeDef* huart, IRManager* irManager);
    
    // 主循环处理
    void process(void);
    
    // 串口回调
    void onUartReceive(uint8_t* buffer, uint16_t size);
    void onUartTxComplete(void);
    
    // 供IRManager调用的接口
    bool pushToSendQueue(uint8_t* data_4bytes);
    
    // 设置发送结果回调
    void setSendResultCallback(SerialSendResultCallback callback);
    
    // 红外发送完成通知（由IRManager调用）
    void onInfraredSendComplete(uint8_t success);
    
    // 标志管理
    volatile uint8_t store_flag;

private:
    SerialProtocol();
    
    // 内部队列操作
    bool popFromSendQueue(uint8_t* data_4bytes);
    bool isSendQueueEmpty(void);
    
    // 协议相关函数
    uint8_t calculateChecksum(uint8_t* data);
    void buildFrame(uint8_t* data, uint8_t parity, uint8_t* frame_out);
    int parseFrame(uint8_t* buffer, uint16_t size, uint8_t* data_out, uint8_t* parity_out);
    void sendFrame(uint8_t* data, uint8_t parity);
    
    // 辅助函数
    uint32_t getTickMs(void);
    void startSending(void);
    void notifySendResult(uint8_t success);
    
private:
    // 硬件相关
    UART_HandleTypeDef* m_huart;
    IRManager* m_irManager;
    
    // 状态机
    SerialState_t m_state;
    SerialSendResultCallback m_resultCallback;
    
    // 发送相关
    uint8_t m_current_send_data[SERIAL_DATA_LEN];
    uint8_t m_current_send_parity;
    uint8_t m_uart_send_frame[SERIAL_FRAME_LEN];
    uint32_t m_send_start_time;          // 当前批次开始时间
    uint32_t m_send_first_start_time;    // 首次发送开始时间（总超时用）
    uint8_t m_send_batch_count;          // 当前批次已发送次数（0-9）
    uint32_t m_retry_count;              // 重发次数（仅记录）
    volatile uint8_t m_tx_complete;      // DMA发送完成标志
    
    // 接收相关
    uint8_t m_rx_buffer[30];
    uint8_t m_ack_frame[SERIAL_FRAME_LEN];
    volatile uint8_t m_rx_ready;
    volatile uint16_t m_rx_size;
    
    // ★ 接收去重变量（基于奇偶标志的永久去重）
    uint8_t m_last_rx_data[SERIAL_DATA_LEN];   // 上次接收的数据
    uint8_t m_last_rx_parity;                  // 上次接收的奇偶标志
    uint32_t m_last_rx_time;                   // 上次接收时间（调试用）
    
    // 等待红外发送完成
    uint8_t m_waiting_ir_complete;
    
    // 发送队列
    uint8_t m_send_queue[SERIAL_QUEUE_SIZE][4];
    volatile uint8_t m_queue_head;
    volatile uint8_t m_queue_tail;
    volatile uint8_t m_queue_count;
		    // ========== 新增：应答控制 ==========
    uint8_t m_last_processed_data[SERIAL_DATA_LEN];   // 上次处理的数据
    uint8_t m_last_processed_parity;                  // 上次处理的奇偶
    uint8_t m_need_send_ack;                          // 需要发送应答
    uint8_t m_ack_sent;                               // 应答已发送
    uint8_t m_ack_retry_count;                        // 应答重发次数（调试用）
    
    void sendOneAckFrame(void);  // 发送一次应答
		
};

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PROTOCOL_H */