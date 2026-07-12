/** 
  ******************************************************************************
  * @file    quad_encoder.c
  * @brief   TIM1 CH1(PE9)/CH2(PE11) 软件输入捕获正交编码器解码实现
  * 
  * 说明：
  *   1. 利用 CubeMX 已配置的 TIM1 CH1/CH2 双边沿输入捕获产生中断。
  *   2. 中断中读取当前 A/B 电平，与上次状态比较，实现 4x 正交解码。
  *   3. 相邻两次有效计数变化使用 TIM1 捕获值计算间隔，实现 T 法测速。
  *   4. HAL_TIM_IC_CaptureCallback 为弱函数，本文件直接覆盖；
  *      若未来其它定时器也需要输入捕获回调，请改为回调分发方式。
  ******************************************************************************
  */
#include "quad_encoder.h"
#include "tim.h"

static QuadEncoder_TypeDef g_qe = {0};

/**
  * @brief  读取当前 AB 状态，编码为 (A << 1) | B
  * @retval 0/1/2/3
  */
static inline uint8_t QE_ReadState(void)
{
    uint8_t a = (HAL_GPIO_ReadPin(QE_CH_A_PORT, QE_CH_A_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t b = (HAL_GPIO_ReadPin(QE_CH_B_PORT, QE_CH_B_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    return (uint8_t)((a << 1U) | b);
}

/**
  * @brief  根据正交编码器状态转移判断方向
  * @note   正转（定义）序列：00 -> 01 -> 11 -> 10 -> 00
  *         反转（定义）序列：00 -> 10 -> 11 -> 01 -> 00
  *         若实际方向相反，可交换 QE_CH_A_PIN/QE_CH_B_PIN 或在应用中取反。
  */
static inline int8_t QE_GetDirection(uint8_t prev, uint8_t curr)
{
    if (prev == curr)
    {
        return 0;
    }

    if (((prev == 0U) && (curr == 1U)) ||
        ((prev == 1U) && (curr == 3U)) ||
        ((prev == 3U) && (curr == 2U)) ||
        ((prev == 2U) && (curr == 0U)))
    {
        return 1;   /* 正转 */
    }

    if (((prev == 0U) && (curr == 2U)) ||
        ((prev == 2U) && (curr == 3U)) ||
        ((prev == 3U) && (curr == 1U)) ||
        ((prev == 1U) && (curr == 0U)))
    {
        return -1;  /* 反转 */
    }

    return 0;       /* 非法跳变（噪声/抖动），忽略 */
}

void QuadEncoder_Init(void)
{
    g_qe.count            = 0;
    g_qe.last_count       = 0;
    g_qe.pulse_us         = 0U;
    g_qe.last_capture     = 0U;
    g_qe.overflow         = 0U;
    g_qe.last_state       = QE_ReadState();
    g_qe.direction        = 0;
    g_qe.first_pulse      = 1U;
    g_qe.pulse_timeout_us = QE_PULSE_TIMEOUT_US;

    /* 启动 TIM1 CH1/CH2 输入捕获中断；双边沿触发已由 CubeMX 配置 */
    HAL_TIM_IC_Start_IT(QE_TIM_HANDLE, QE_CH_A_NUM);
    HAL_TIM_IC_Start_IT(QE_TIM_HANDLE, QE_CH_B_NUM);
}

/**
  * @brief  TIM1 输入捕获中断回调（覆盖 HAL 弱函数）
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t capture;
    uint8_t  state;
    int8_t   dir;

    if (htim->Instance != QE_TIM)
    {
        return;
    }

    /* 根据触发通道读取对应捕获值 */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        capture = HAL_TIM_ReadCapturedValue(htim, QE_CH_A_NUM);
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        capture = HAL_TIM_ReadCapturedValue(htim, QE_CH_B_NUM);
    }
    else
    {
        return;
    }

    state = QE_ReadState();
    dir   = QE_GetDirection(g_qe.last_state, state);

    if (dir != 0)
    {
        g_qe.count     += dir;
        g_qe.direction  = dir;

        /* T 法测速：用两次有效计数变化的捕获值差计算脉冲间隔 */
        if (g_qe.first_pulse != 0U)
        {
            g_qe.first_pulse  = 0U;
            g_qe.last_capture = capture;
            g_qe.overflow     = 0U;
        }
        else
        {
            uint32_t delta_tick = capture + (g_qe.overflow * 65536U) - g_qe.last_capture;
            g_qe.pulse_us       = delta_tick * QE_TIM_TICK_US;
            g_qe.last_capture   = capture;
            g_qe.overflow       = 0U;
        }
    }

    g_qe.last_state = state;
}

/**
  * @brief  TIM 更新中断回调辅助函数，用于扩展 T 法计时范围
  */
void QuadEncoder_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == QE_TIM)
    {
        g_qe.overflow++;
    }
}

int32_t QuadEncoder_GetCount(void)
{
    return g_qe.count;
}

void QuadEncoder_SetCount(int32_t value)
{
    g_qe.count = value;
}

int32_t QuadEncoder_GetDeltaCount(void)
{
    int32_t delta = g_qe.count - g_qe.last_count;
    g_qe.last_count = g_qe.count;
    return delta;
}

uint32_t QuadEncoder_GetPulseUs(void)
{
    return g_qe.pulse_us;
}

int8_t QuadEncoder_GetDirection(void)
{
    return g_qe.direction;
}

float QuadEncoder_GetSpeed_RPM(uint16_t ppr)
{
    uint32_t pulse_us = g_qe.pulse_us;

    if ((pulse_us == 0U) || (pulse_us > g_qe.pulse_timeout_us) || (ppr == 0U))
    {
        return 0.0f;
    }

    /* 4x 解码：每转计数 = 4 * ppr */
    float rpm = 15.0e6f / ((float)ppr * (float)pulse_us);
    return (g_qe.direction > 0) ? rpm : -rpm;
}

float QuadEncoder_GetSpeed_Rads(uint16_t ppr)
{
    float rpm = QuadEncoder_GetSpeed_RPM(ppr);
    return rpm * 2.0f * 3.1415926f / 60.0f;
}

void QuadEncoder_SetPulseTimeout(uint32_t timeout_us)
{
    g_qe.pulse_timeout_us = timeout_us;
}
