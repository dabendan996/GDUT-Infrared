#ifndef MODULE_IRED_H
#define MODULE_IRED_H

#include "main.h"
#include "tim.h"
#include "dma.h"
#include "BSP_TimeStamp.h"
#include "string.h"

// 前向声明
class SerialProtocol;

/* 常量定义 */
#define LENGTH 70
#define CAPTURE_TIMEOUT_US  700000

#ifdef __cplusplus
extern "C" {
#endif

void DWT_Init(void);
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

/* 状态枚举 */
typedef enum 
{
    Send,
    Receive_Respon,
    Receive
} Data_State;

typedef enum 
{
    R1,
    R2
} Ired_Type;

extern uint8_t uart2_data_rec[30];
extern uint8_t uart2_data_snd[30];

/* 红外管理类 */
class IRManager {
public:
    // 构造函数
    IRManager(uint8_t Send_Data_Type);
    
    // 初始化方法
    void init(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim4, TIM_HandleTypeDef* htim3,
              TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim8);
    
    // 获取接收到的数据
    void getReceivedData(uint8_t* data_out);
    
    // 设置响应数据
    void setResponseData(uint8_t* data);
    
    // 设置发送数据
    void setSendData(uint8_t* data);
    
    // 红外发送
    void IR_Send(uint8_t* data, uint8_t reset_timeout);
    
    // 捕获控制
    void Disable_Capture_And_DMA(void);
    void Enable_Capture_And_DMA(void);
    
    // 主循环处理
    void process(void);
    
    // 注册回调函数
    void setOnDataReceived(void (*callback)(uint8_t* data, uint8_t len)) {
        m_onDataReceived = callback;
    }

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
    
    // 获取状态
    Data_State getState(void) const;
    
    // 获取 store_flag
    volatile uint8_t getStoreFlag(void);
    
    // 获取 uart2_data_snd（保持原接口）
    uint8_t* getUart2DataSnd(void);
    
    // 重置发送计数器
    void resetSendCounter(void);
    
    void ProcessCaptureCallback(TIM_HandleTypeDef* htim);
    
    // 设置自动响应
    void setAutoResponse(bool enable);
    bool hasPendingData(void);
		void Clear_Data(void);
    
public:
    // 公开原有变量（保持与原代码兼容）
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
    uint8_t Send_Data_cnt=0;
    uint8_t cnt=0;
    volatile uint8_t store_flag=0;
    volatile uint8_t rec_suc=0;
    uint8_t Send_Data[3];
    uint8_t Respon_Data[3];
    volatile Data_State state=Receive;
    volatile uint32_t sum=0;
    uint8_t cnt_re=0;
    uint8_t Data1[LENGTH];
    uint8_t Data2[LENGTH];
    uint16_t Data3[LENGTH];
    uint16_t count=0;
    uint16_t temp=0;
    uint8_t send_cnt=0;
    uint8_t Start_Send_delay=0;
    uint64_t last_send_delay_us=0;
    volatile uint64_t last_capture_tick_us=0;
    volatile uint8_t capture_active=1;
    volatile uint8_t need_to_enable_capture=0;
    volatile uint8_t need_to_disable_capture=0;
    volatile uint8_t ir_complete_processed;
    Ired_Type Ired_Type_Get()
    {
        if (Send_Data_Type==0)
            Ired= R1;
        else if(Send_Data_Type==1)
            Ired= R2;
        return Ired;
    };
    uint8_t Data_cnt_flag=0;
    
private:
	 volatile uint8_t need_notify_ir_complete;
    // 定时器句柄
    TIM_HandleTypeDef* htim2_ptr=nullptr;
    TIM_HandleTypeDef* htim4_ptr=nullptr;
    TIM_HandleTypeDef* htim3_ptr=nullptr;
    TIM_HandleTypeDef* htim1_ptr=nullptr;
    TIM_HandleTypeDef* htim8_ptr=nullptr;
    uint8_t Send_Data_Type;
    Ired_Type Ired;
    
    // 回调函数
    void (*m_onDataReceived)(uint8_t* data, uint8_t len);
    bool m_auto_response;
    
    // 私有方法
    void Period_Measure_Init(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t* Data);
    void Red_Ray_DATA_Procss(uint32_t* Data, HAL_TIM_ActiveChannel channel);
    void IR_SendLeader(void);
    void IR_SendBit(uint8_t bit);
    void IR_WriteByte(uint8_t byte);
    
    // 队列相关
    static const uint8_t QUEUE_SIZE = 10;
    uint8_t send_queue[QUEUE_SIZE][4];
    volatile uint8_t queue_head;
    volatile uint8_t queue_tail;
    volatile uint8_t queue_count;
    
    // 数据去重
    uint8_t last_processed_data[4];
    volatile uint8_t last_data_valid;
    bool in_window=0;
    
    // 窗口管理
    uint64_t last_response_end_time;
    uint8_t last_received_command[4];
    uint8_t pending_command[4];
    uint8_t has_pending_command;
    
    // 新增方法
    void handleNewData(uint8_t* new_data);
    bool isDataSame(uint8_t* data1, uint8_t* data2);
    void storeToPendingAndCheck(uint8_t* data);
    void checkAndSendPendingData(void);
};

#endif /* IR_MANAGER_H */