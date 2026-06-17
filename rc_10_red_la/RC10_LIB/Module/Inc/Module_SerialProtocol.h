// SerialProtocol.h
#ifndef MODULE_SERIAL_PROTOCOL_H
#define MODULE_SERIAL_PROTOCOL_H

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
    static SerialProtocol* getInstance();
    
    // 初始化
    void init(UART_HandleTypeDef* huart, IRManager* irManager);
    
    // 主循环处理
    void process(void);
    
    // 串口回调
    void onUartReceive(uint8_t* buffer, uint16_t size);
    void checkAndRestartUart(void);
    // 供IRManager调用的接口
    bool pushToSendQueue(uint8_t* data_4bytes);
    uint8_t m_rx_buffer[30];    
    volatile uint16_t m_rx_size;
    
    // 标志管理
    volatile uint8_t store_flag;
    volatile uint8_t m_rx_ready;

private:
    SerialProtocol();
    
    // 协议相关函数
    uint8_t calculateChecksum(uint8_t* data);
    int parseFrame(uint8_t* buffer, uint16_t size, uint8_t* data_out, uint8_t* parity_out); 
private:
    // 硬件相关
    UART_HandleTypeDef* m_huart;
    IRManager* m_irManager;
    
    // 状态机
    SerialState_t m_state;
    SerialSendResultCallback m_resultCallback;
    // 接收相关
//    uint8_t m_rx_buffer[30];
//    volatile uint16_t m_rx_size;
    
    // ★ 接收去重变量（基于奇偶标志的永久去重）
    uint8_t m_last_rx_data[SERIAL_DATA_LEN];   // 上次接收的数据
    uint8_t m_last_rx_parity;                  // 上次接收的奇偶标志
    uint32_t m_last_rx_time;                   // 上次接收时间（调试用）
		
};

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PROTOCOL_H */