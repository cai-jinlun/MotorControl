/** 
  ******************************************************************************
  * @file    quad_encoder.h
  * @brief   TIM1 CH1(PE9)/CH2(PE11) 软件输入捕获正交编码器解码头文件
  *          - 手动判断 A/B 相边沿变化并计数（4x 解码）
  *          - T 法测速：测量相邻有效计数变化的时间间隔
  ******************************************************************************
  */
#ifndef __QUAD_ENCODER_H
#define __QUAD_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 编码器硬件连接（与 main.h 中定义的引脚保持一致） */
#define QE_TIM                TIM1
#define QE_TIM_HANDLE         (&htim1)
#define QE_CH_A_PIN           H1_Hall_A_Pin
#define QE_CH_A_PORT          H1_Hall_A_GPIO_Port
#define QE_CH_A_NUM           TIM_CHANNEL_1
#define QE_CH_B_PIN           H1_Hall_B_Pin
#define QE_CH_B_PORT          H1_Hall_B_GPIO_Port
#define QE_CH_B_NUM           TIM_CHANNEL_2

/* TIM1 配置：168 MHz / 16800 = 10 kHz，1 tick = 100 us */
#define QE_TIM_TICK_US        100U

/*定时器period */
#define QE_TIM_PERIOD         1000U

/* 默认堵转超时：超过该时间未收到脉冲认为速度为 0，单位 us */
#define QE_PULSE_TIMEOUT_US   1000000U

typedef struct
{
    int32_t  count;              /* 32 位编码器计数值 */
    int32_t  last_count;         /* 上次读取的计数值，用于 M 法测速 */
    uint32_t pulse;              /* 相邻两次有效计数变化的间隔（定时器计数值） */
    uint32_t last_capture;       /* 上一次捕获的定时器值 */
    uint32_t overflow;           /* 两次捕获之间 TIM1 溢出次数 */
    uint8_t  last_state;         /* 上一次 AB 状态：bit1=A，bit0=B */
    int8_t   direction;          /* 最近一次变化方向：+1 正转，-1 反转，0 无变化 */
    uint8_t  first_pulse;        /* 1=尚未获得第二个有效脉冲（T 法未就绪） */
    uint32_t pulse_timeout;      /* 堵转判定阈值 */
    float    velocity;           /* 速度（单位：每秒脉冲数） */
} QuadEncoder_TypeDef;

/* 初始化并启动 TIM1 CH1/CH2 输入捕获中断 */
void QuadEncoder_Init(void);

/* 计数器读写 */
int32_t  QuadEncoder_GetCount(void);
void     QuadEncoder_SetCount(int32_t value);

/* M 法测速：获取本次调用与上次调用之间的计数差，并重置基准 */
int32_t  QuadEncoder_GetDeltaCount(void);

/* T 法测速 */
uint32_t QuadEncoder_GetPulseUs(void);        /* 两次有效计数变化的间隔（us） */
int8_t   QuadEncoder_GetDirection(void);      /* 最近一次转动方向 */

/* 设置堵转超时时间（us），超过该时间未收到脉冲 GetSpeed_xxx 返回 0 */
void     QuadEncoder_SetPulseTimeout(uint32_t timeout_us);

/* 
 * 需在 main.c 的 HAL_TIM_PeriodElapsedCallback 中调用，
 * 用于 T 法测速时记录 TIM1 溢出次数。
 */
void     QuadEncoder_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_ENCODER_H */
