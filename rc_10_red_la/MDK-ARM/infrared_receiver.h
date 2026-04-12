#ifndef __INFRARED_RECEIVER_H
#define __INFRARED_RECEIVER_H

#include "stm32h7xx_hal.h"
#include "stdint.h"
#include "tim.h"
typedef enum {
    IR_PROTOCOL_NEC = 0,      // NEC协议
    IR_PROTOCOL_SONY,         // SONY协议
    IR_PROTOCOL_RC5,          // RC5协议
    IR_PROTOCOL_RC6,          // RC6协议
    IR_PROTOCOL_UNKNOWN       // 未知协议
} IR_Protocol_t;

/* NEC协议时间常量（单位：微秒） */
#define NEC_LEADER_LOW_MIN    8000    // 引导码低电平最小时间
#define NEC_LEADER_LOW_MAX    10000   // 引导码低电平最大时间
#define NEC_LEADER_HIGH_MIN   3500    // 引导码高电平最小时间
#define NEC_LEADER_HIGH_MAX   5500    // 引导码高电平最大时间
#define NEC_REPEAT_HIGH_MIN   1800    // 重复码高电平最小时间
#define NEC_REPEAT_HIGH_MAX   2800    // 重复码高电平最大时间
#define NEC_BIT_0_MIN         800     // 逻辑0高电平最小时间
#define NEC_BIT_0_MAX         1400    // 逻辑0高电平最大时间
#define NEC_BIT_1_MIN         1800    // 逻辑1高电平最小时间
#define NEC_BIT_1_MAX         2500    // 逻辑1高电平最大时间
#define NEC_BIT_LOW_MIN       300     // 数据位低电平最小时间
#define NEC_BIT_LOW_MAX       800     // 数据位低电平最大时间

/* 用户回调函数类型 */
typedef void (*IR_DataCallback_t)(uint8_t address, uint8_t command, uint8_t repeat);
typedef void (*IR_ErrorCallback_t)(uint8_t error_code);

/* 接收器错误码 */
typedef enum {
    IR_ERROR_NONE = 0,
    IR_ERROR_LEADER_LOW,
    IR_ERROR_LEADER_HIGH,
    IR_ERROR_DATA_LOW,
    IR_ERROR_DATA_HIGH,
    IR_ERROR_CHECKSUM,
    IR_ERROR_TIMEOUT,
    IR_ERROR_NOISE
} IR_Error_t;

/* 红外接收初始化结构体 */
typedef struct {
    TIM_HandleTypeDef*   htim;           // 定时器句柄
    uint32_t             channel;        // 定时器通道
    uint8_t              protocol;       // 协议类型
    uint16_t             timeout_ms;     // 接收超时时间
    uint8_t              enable_repeat;  // 是否使能重复码
    IR_DataCallback_t    data_callback;  // 数据回调函数
    IR_ErrorCallback_t   error_callback; // 错误回调函数
} IR_Receiver_Config_t;

/* 红外接收器句柄 */
typedef struct {
    IR_Receiver_Config_t config;         // 配置参数
    
    /* 接收状态 */
    uint8_t  state;                      // 当前状态
    uint32_t last_capture;               // 上次捕获值
    uint8_t  last_edge;                  // 上次边沿类型（0:上升沿，1:下降沿）
    uint32_t pulse_width;                // 当前脉冲宽度
    
    /* NEC协议相关 */
    uint32_t data;                       // 接收的数据
    uint8_t  bit_count;                  // 已接收位数
    uint8_t  address;                    // 地址码
    uint8_t  command;                    // 命令码
    uint8_t  repeat_flag;                // 重复码标志
    
    /* 性能统计 */
    uint32_t receive_count;              // 成功接收次数
    uint32_t error_count;                // 错误次数
    uint32_t repeat_count;               // 重复码次数
    uint32_t last_receive_time;          // 上次接收成功时间
    
    /* 超时控制 */
    uint32_t start_time;                 // 接收开始时间
    uint8_t  timeout_flag;               // 超时标志
} IR_Receiver_Handle_t;

/* 状态机状态定义 */
typedef enum {
    IR_STATE_IDLE = 0,           // 空闲状态
    IR_STATE_LEADER_LOW,         // 等待引导码低电平
    IR_STATE_LEADER_HIGH,        // 等待引导码高电平
    IR_STATE_DATA_LOW,           // 等待数据位低电平
    IR_STATE_DATA_HIGH,          // 等待数据位高电平
    IR_STATE_COMPLETE,           // 接收完成
    IR_STATE_ERROR               // 错误状态
} IR_State_t;

/* 公共函数声明 */
void IR_Receiver_Init(IR_Receiver_Handle_t* hrec, IR_Receiver_Config_t* config);
void IR_Receiver_Start(IR_Receiver_Handle_t* hrec);
void IR_Receiver_Stop(IR_Receiver_Handle_t* hrec);
void IR_Receiver_Process(IR_Receiver_Handle_t* hrec);
uint8_t IR_Receiver_IsBusy(IR_Receiver_Handle_t* hrec);
void IR_Receiver_Reset(IR_Receiver_Handle_t* hrec);
void IR_Receiver_GetStats(IR_Receiver_Handle_t* hrec, 
                          uint32_t* receive_count, 
                          uint32_t* error_count,
                          uint32_t* repeat_count);

/* 输入捕获中断回调函数（需在stm32f1xx_it.c中调用） */
void IR_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim);
#endif