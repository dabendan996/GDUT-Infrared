#include "infrared_receiver.h"

#include "infrared_receiver.h"
#include <string.h>

/* 静态全局变量 */
static IR_Receiver_Handle_t* ir_handle = NULL;

/* 私有函数声明 */
static uint8_t IR_IsValidNECLeaderLow(uint32_t pulse_width);
static uint8_t IR_IsValidNECLeaderHigh(uint32_t pulse_width);
static uint8_t IR_IsNECRepeatCode(uint32_t pulse_width);
static uint8_t IR_IsValidNECBitLow(uint32_t pulse_width);
static uint8_t IR_IsValidNECBitHigh(uint32_t pulse_width);
static uint8_t IR_IsNECBitOne(uint32_t pulse_width);
static uint8_t IR_VerifyNECData(uint32_t data, uint8_t* address, uint8_t* command);
static void IR_ProcessPulse(IR_Receiver_Handle_t* hrec, uint32_t pulse_width, uint8_t edge_type);
static void IR_ResetState(IR_Receiver_Handle_t* hrec);
static void IR_HandleTimeout(IR_Receiver_Handle_t* hrec);
static void IR_ProcessNECPulse(IR_Receiver_Handle_t* hrec, uint32_t pulse_width, uint8_t edge_type);
/**
  * @brief  初始化红外接收器
  * @param  hrec: 红外接收器句柄指针
  * @param  config: 配置参数指针
  * @retval 无
  */
void IR_Receiver_Init(IR_Receiver_Handle_t* hrec, IR_Receiver_Config_t* config)
{
    /* 检查参数 */
    if (hrec == NULL || config == NULL || config->htim == NULL) {
        return;
    }
    
    /* 复制配置 */
    memcpy(&hrec->config, config, sizeof(IR_Receiver_Config_t));
    
    /* 初始化状态 */
    IR_ResetState(hrec);
    
    /* 初始化统计信息 */
    hrec->receive_count = 0;
    hrec->error_count = 0;
    hrec->repeat_count = 0;
    hrec->last_receive_time = 0;
    
    /* 设置全局句柄（用于中断回调） */
    ir_handle = hrec;
    
    /* 配置定时器输入捕获 */
    TIM_IC_InitTypeDef sConfigIC = {0};
    
    /* 初始配置为下降沿捕获 */
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0x3;  /* 4个时钟周期滤波 */
    
    HAL_TIM_IC_ConfigChannel(hrec->config.htim, &sConfigIC, hrec->config.channel);
}

/**
  * @brief  启动红外接收
  * @param  hrec: 红外接收器句柄指针
  * @retval 无
  */
void IR_Receiver_Start(IR_Receiver_Handle_t* hrec)
{
    if (hrec == NULL || hrec->config.htim == NULL) {
        return;
    }
    
    /* 重置状态 */
    IR_ResetState(hrec);
    
    /* 启动输入捕获中断 */
    HAL_TIM_IC_Start_IT(hrec->config.htim, hrec->config.channel);
    
    /* 记录开始时间（用于超时检测） */
    hrec->start_time = HAL_GetTick();
    hrec->timeout_flag = 0;
}

/**
  * @brief  停止红外接收
  * @param  hrec: 红外接收器句柄指针
  * @retval 无
  */
void IR_Receiver_Stop(IR_Receiver_Handle_t* hrec)
{
    if (hrec == NULL || hrec->config.htim == NULL) {
        return;
    }
    
    /* 停止输入捕获 */
    HAL_TIM_IC_Stop_IT(hrec->config.htim, hrec->config.channel);
    
    /* 重置状态 */
    hrec->state = IR_STATE_IDLE;
}

/**
  * @brief  处理接收完成的数据（在主循环中调用）
  * @param  hrec: 红外接收器句柄指针
  * @retval 无
  */
void IR_Receiver_Process(IR_Receiver_Handle_t* hrec)
{
    if (hrec == NULL) {
        return;
    }
    
    /* 检查超时 */
    if (hrec->state != IR_STATE_IDLE && hrec->state != IR_STATE_COMPLETE) {
        uint32_t current_time = HAL_GetTick();
        if ((current_time - hrec->start_time) > hrec->config.timeout_ms) {
            IR_HandleTimeout(hrec);
        }
    }
    
    /* 处理接收完成状态 */
    if (hrec->state == IR_STATE_COMPLETE) {
        /* 调用数据回调函数 */
        if (hrec->config.data_callback != NULL) {
            hrec->config.data_callback(hrec->address, hrec->command, hrec->repeat_flag);
        }
        
        /* 更新统计信息 */
        if (hrec->repeat_flag) {
            hrec->repeat_count++;
        } else {
            hrec->receive_count++;
        }
        hrec->last_receive_time = HAL_GetTick();
        
        /* 重置状态 */
        hrec->state = IR_STATE_IDLE;
        hrec->repeat_flag = 0;
    }
    
    /* 处理错误状态 */
    if (hrec->state == IR_STATE_ERROR) {
        /* 调用错误回调函数 */
        if (hrec->config.error_callback != NULL) {
            // 这里可以传递具体的错误码，简化实现
            hrec->config.error_callback(IR_ERROR_TIMEOUT);
        }
        
        /* 更新错误计数 */
        hrec->error_count++;
        
        /* 重置状态 */
        hrec->state = IR_STATE_IDLE;
    }
}

/**
  * @brief  检查接收器是否忙
  * @param  hrec: 红外接收器句柄指针
  * @retval 1: 忙，0: 空闲
  */
uint8_t IR_Receiver_IsBusy(IR_Receiver_Handle_t* hrec)
{
    if (hrec == NULL) {
        return 0;
    }
    
    return (hrec->state != IR_STATE_IDLE);
}

/**
  * @brief  重置接收器状态
  * @param  hrec: 红外接收器句柄指针
  * @retval 无
  */
void IR_Receiver_Reset(IR_Receiver_Handle_t* hrec)
{
    IR_ResetState(hrec);
    hrec->error_count = 0;
}

/**
  * @brief  获取统计信息
  * @param  hrec: 红外接收器句柄指针
  * @param  receive_count: 成功接收次数指针
  * @param  error_count: 错误次数指针
  * @param  repeat_count: 重复码次数指针
  * @retval 无
  */
void IR_Receiver_GetStats(IR_Receiver_Handle_t* hrec, 
                          uint32_t* receive_count, 
                          uint32_t* error_count,
                          uint32_t* repeat_count)
{
    if (hrec != NULL) {
        if (receive_count != NULL) *receive_count = hrec->receive_count;
        if (error_count != NULL) *error_count = hrec->error_count;
        if (repeat_count != NULL) *repeat_count = hrec->repeat_count;
    }
}

/**
  * @brief  输入捕获中断回调函数
  * @param  htim: 定时器句柄指针
  * @retval 无
  */
void IR_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
    if (ir_handle == NULL || ir_handle->config.htim != htim) {
        return;
    }
    
    uint32_t current_capture;
    uint32_t pulse_width;
    
    /* 读取当前捕获值 */
    current_capture = HAL_TIM_ReadCapturedValue(htim, ir_handle->config.channel);
    
    /* 计算脉冲宽度（处理计数器溢出） */
    if (current_capture >= ir_handle->last_capture) {
        pulse_width = current_capture - ir_handle->last_capture;
    } else {
        /* 计数器溢出 */
        pulse_width = (0xFFFF - ir_handle->last_capture) + current_capture + 1;
    }
    
    /* 更新上次捕获值 */
    ir_handle->last_capture = current_capture;
    
    /* 处理脉冲 */
    IR_ProcessPulse(ir_handle, pulse_width, ir_handle->last_edge);
    
    /* 切换捕获边沿 */
    if (ir_handle->last_edge == 0) {
        /* 当前是下降沿捕获，切换为上升沿 */
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, ir_handle->config.channel, 
                                     TIM_INPUTCHANNELPOLARITY_RISING);
        ir_handle->last_edge = 1;
    } else {
        /* 当前是上升沿捕获，切换为下降沿 */
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, ir_handle->config.channel, 
                                     TIM_INPUTCHANNELPOLARITY_FALLING);
        ir_handle->last_edge = 0;
    }
}

/* 私有函数实现 */

/**
  * @brief  处理脉冲数据
  * @param  hrec: 红外接收器句柄指针
  * @param  pulse_width: 脉冲宽度
  * @param  edge_type: 边沿类型（0:下降沿，1:上升沿）
  * @retval 无
  */
static void IR_ProcessPulse(IR_Receiver_Handle_t* hrec, uint32_t pulse_width, uint8_t edge_type)
{
    /* 根据协议类型处理 */
    if (hrec->config.protocol == IR_PROTOCOL_NEC) {
        IR_ProcessNECPulse(hrec, pulse_width, edge_type);
    }
    /* 可以扩展其他协议 */
}

/**
  * @brief  处理NEC协议脉冲
  * @param  hrec: 红外接收器句柄指针
  * @param  pulse_width: 脉冲宽度
  * @param  edge_type: 边沿类型（0:下降沿，1:上升沿）
  * @retval 无
  */
static void IR_ProcessNECPulse(IR_Receiver_Handle_t* hrec, uint32_t pulse_width, uint8_t edge_type)
{
    switch (hrec->state) {
        case IR_STATE_IDLE:
            /* 空闲状态，等待引导码开始（下降沿） */
            if (edge_type == 0 && IR_IsValidNECLeaderLow(pulse_width)) {
                /* 检测到引导码开始，进入等待引导码高电平状态 */
                hrec->state = IR_STATE_LEADER_HIGH;
                hrec->start_time = HAL_GetTick();  /* 重置超时计时 */
                hrec->timeout_flag = 0;
            }
            break;
            
        case IR_STATE_LEADER_HIGH:
            /* 等待引导码高电平（上升沿） */
            if (edge_type == 1) {
                if (IR_IsValidNECLeaderHigh(pulse_width)) {
                    /* 正常引导码，开始接收数据 */
                    hrec->state = IR_STATE_DATA_LOW;
                    hrec->data = 0;
                    hrec->bit_count = 0;
                    hrec->repeat_flag = 0;
                } else if (IR_IsNECRepeatCode(pulse_width)) {
                    /* 重复码 */
                    if (hrec->config.enable_repeat) {
                        hrec->repeat_flag = 1;
                        hrec->state = IR_STATE_COMPLETE;
                    } else {
                        hrec->state = IR_STATE_IDLE;
                    }
                } else {
                    /* 引导码错误 */
                    hrec->state = IR_STATE_ERROR;
                    if (hrec->config.error_callback != NULL) {
                        hrec->config.error_callback(IR_ERROR_LEADER_HIGH);
                    }
                }
            }
            break;
            
        case IR_STATE_DATA_LOW:
            /* 等待数据位低电平结束（上升沿） */
            if (edge_type == 1) {
                if (IR_IsValidNECBitLow(pulse_width)) {
                    /* 有效的低电平，等待高电平 */
                    hrec->state = IR_STATE_DATA_HIGH;
                } else {
                    /* 低电平错误 */
                    hrec->state = IR_STATE_ERROR;
                    if (hrec->config.error_callback != NULL) {
                        hrec->config.error_callback(IR_ERROR_DATA_LOW);
                    }
                }
            }
            break;
            
        case IR_STATE_DATA_HIGH:
            /* 等待数据位高电平结束（下降沿） */
            if (edge_type == 0) {
                if (IR_IsValidNECBitHigh(pulse_width)) {
                    /* 接收数据位 */
                    hrec->data >>= 1;  /* 为新的位腾出空间 */
                    
                    if (IR_IsNECBitOne(pulse_width)) {
                        /* 逻辑1 */
                        hrec->data |= 0x80000000;
                    }
                    /* 逻辑0：高位保持0 */
                    
                    hrec->bit_count++;
                    
                    /* 检查是否接收完32位 */
                    if (hrec->bit_count >= 32) {
                        /* 验证数据 */
                        if (IR_VerifyNECData(hrec->data, &hrec->address, &hrec->command)) {
                            hrec->state = IR_STATE_COMPLETE;
                        } else {
                            /* 数据校验失败 */
                            hrec->state = IR_STATE_ERROR;
                            if (hrec->config.error_callback != NULL) {
                                hrec->config.error_callback(IR_ERROR_CHECKSUM);
                            }
                        }
                    } else {
                        /* 继续接收下一位 */
                        hrec->state = IR_STATE_DATA_LOW;
                    }
                } else {
                    /* 高电平错误 */
                    hrec->state = IR_STATE_ERROR;
                    if (hrec->config.error_callback != NULL) {
                        hrec->config.error_callback(IR_ERROR_DATA_HIGH);
                    }
                }
            }
            break;
            
        default:
            break;
    }
}

/**
  * @brief  检查是否为有效的NEC引导码低电平
  * @param  pulse_width: 脉冲宽度（微秒）
  * @retval 1: 有效，0: 无效
  */
static uint8_t IR_IsValidNECLeaderLow(uint32_t pulse_width)
{
    return (pulse_width >= NEC_LEADER_LOW_MIN && 
            pulse_width <= NEC_LEADER_LOW_MAX);
}

/**
  * @brief  检查是否为有效的NEC引导码高电平
  * @param  pulse_width: 脉冲宽度（微秒）
  * @retval 1: 有效，0: 无效
  */
static uint8_t IR_IsValidNECLeaderHigh(uint32_t pulse_width)
{
    return (pulse_width >= NEC_LEADER_HIGH_MIN && 
            pulse_width <= NEC_LEADER_HIGH_MAX);
}

/**
  * @brief  检查是否为NEC重复码
  * @param  pulse_width: 脉冲宽度（微秒）
  * @retval 1: 是重复码，0: 不是重复码
  */
static uint8_t IR_IsNECRepeatCode(uint32_t pulse_width)
{
    return (pulse_width >= NEC_REPEAT_HIGH_MIN && 
            pulse_width <= NEC_REPEAT_HIGH_MAX);
}

/**
  * @brief  检查是否为有效的NEC数据位低电平
  * @param  pulse_width: 脉冲宽度（微秒）
  * @retval 1: 有效，0: 无效
  */
static uint8_t IR_IsValidNECBitLow(uint32_t pulse_width)
{
	return (pulse_width >= NEC_BIT_LOW_MIN && 
					pulse_width <= NEC_BIT_LOW_MAX);
}

/**
  * @brief  检查是否为有效的NEC数据位高电平
  * @param  pulse_width: 脉冲宽度（微秒）
  * @retval 1: 有效，0: 无效
  */
static uint8_t IR_IsValidNECBitHigh(uint32_t pulse_width)
{
    return (pulse_width >= NEC_BIT_0_MIN && 
            pulse_width <= NEC_BIT_1_MAX);
}

/**
  * @brief  检查NEC数据位是否为逻辑1
  * @param  pulse_width: 脉冲宽度（微秒）
  * @retval 1: 逻辑1，0: 逻辑0
  */
static uint8_t IR_IsNECBitOne(uint32_t pulse_width)
{
    return (pulse_width >= NEC_BIT_1_MIN && 
            pulse_width <= NEC_BIT_1_MAX);
}

/**
  * @brief  验证NEC数据并提取地址和命令
  * @param  data: 接收的32位数据
  * @param  address: 提取的地址码指针
  * @param  command: 提取的命令码指针
  * @retval 1: 验证成功，0: 验证失败
  */
static uint8_t IR_VerifyNECData(uint32_t data, uint8_t* address, uint8_t* command)
{
    uint8_t addr = (data >> 24) & 0xFF;
    uint8_t addr_inv = (data >> 16) & 0xFF;
    uint8_t cmd = (data >> 8) & 0xFF;
    uint8_t cmd_inv = data & 0xFF;
    
    /* 检查反码 */
    if ((addr + addr_inv == 0xFF) && (cmd + cmd_inv == 0xFF)) {
        if (address != NULL) *address = addr;
        if (command != NULL) *command = cmd;
        return 1;
    }
    
    return 0;
}

/**
  * @brief  重置接收器状态
  * @param  hrec: 红外接收器句柄指针
  * @retval 无
  */
static void IR_ResetState(IR_Receiver_Handle_t* hrec)
{
    hrec->state = IR_STATE_IDLE;
    hrec->last_capture = 0;
    hrec->last_edge = 0;  /* 初始为下降沿捕获 */
    hrec->pulse_width = 0;
    hrec->data = 0;
    hrec->bit_count = 0;
    hrec->address = 0;
    hrec->command = 0;
    hrec->repeat_flag = 0;
    hrec->start_time = 0;
    hrec->timeout_flag = 0;
}

/**
  * @brief  处理超时
  * @param  hrec: 红外接收器句柄指针
  * @retval 无
  */
static void IR_HandleTimeout(IR_Receiver_Handle_t* hrec)
{
    if (!hrec->timeout_flag) {
        hrec->timeout_flag = 1;
        hrec->state = IR_STATE_ERROR;
        if (hrec->config.error_callback != NULL) {
            hrec->config.error_callback(IR_ERROR_TIMEOUT);
        }
    }
}