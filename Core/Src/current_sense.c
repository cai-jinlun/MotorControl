#include "current_sense.h"
#include "adc.h"
#include "tim.h"

#define CURRENT_SENSE_HALF_0_VALID  (1U << 0)
#define CURRENT_SENSE_HALF_1_VALID  (1U << 1)
#define CURRENT_SENSE_ALL_VALID     (CURRENT_SENSE_HALF_0_VALID | \
                                     CURRENT_SENSE_HALF_1_VALID)

/* DMA 持续写入的 32 点循环缓冲区，分为前后两个 16 点半区。 */
static volatile uint16_t s_dma_buffer[CURRENT_SENSE_DMA_BUFFER_SIZE];
/* 两个已完成半区的 ADC 码值之和，用于构造 32 点移动平均。 */
static volatile uint32_t s_half_sum[2];
static volatile uint32_t s_average_sum;  /* 当前有效 32 点窗口的总和 */
static volatile uint16_t s_latest_raw;   /* 最近完成半区的末尾样本 */
static volatile uint8_t s_valid_mask;    /* 两个半区是否均已完成 */
static volatile uint8_t s_started;       /* 采样模块是否已启动 */
static volatile uint8_t s_hw_error;      /* ADC、DMA 或定时器是否发生错误 */

/* 电流换算标定参数。 */
static float s_offset_counts;
static float s_amps_per_count;
static uint8_t s_calibrated;

/* 对指定的已完成 DMA 半区求和；调用时该半区不会被 DMA 写入。 */
static uint32_t CurrentSense_SumHalf(uint32_t start)
{
    uint32_t sum = 0U;
    uint32_t i;

    for (i = start; i < (start + CURRENT_SENSE_DMA_HALF_SIZE); ++i) {
        sum += s_dma_buffer[i];
    }
    return sum;
}

/*
 * 发布一个刚完成半区的数据。
 * 每次更新一个 16 点半区总和；两个半区均有效后，组合成 32 点移动平均。
 */
static void CurrentSense_PublishHalf(uint32_t half)
{
    uint32_t start = half * CURRENT_SENSE_DMA_HALF_SIZE;
    uint32_t sum = CurrentSense_SumHalf(start);
    uint8_t valid_bit = (half == 0U) ? CURRENT_SENSE_HALF_0_VALID
                                     : CURRENT_SENSE_HALF_1_VALID;
    uint8_t new_valid_mask = s_valid_mask | valid_bit;

    s_half_sum[half] = sum;
    s_latest_raw = s_dma_buffer[start + CURRENT_SENSE_DMA_HALF_SIZE - 1U];

    /* 首次形成有效均值后再发布就绪标志，避免读取端拿到未初始化数据。 */
    if (new_valid_mask == CURRENT_SENSE_ALL_VALID) {
        s_average_sum = s_half_sum[0] + s_half_sum[1];
    }
    s_valid_mask = new_valid_mask;
}

/* 初始化采样状态，启动 ADC DMA 与 TIM2_CH1 触发源。重复启动为幂等操作。 */
CurrentSenseStatus_t CurrentSense_Start(void)
{
    uint32_t i;

    if (s_started != 0U) {
        return (s_hw_error != 0U) ? CURRENT_SENSE_ERR_HW_FAILURE
                                  : CURRENT_SENSE_OK;
    }

    for (i = 0U; i < CURRENT_SENSE_DMA_BUFFER_SIZE; ++i) {
        s_dma_buffer[i] = 0U;
    }
    s_half_sum[0] = 0U;
    s_half_sum[1] = 0U;
    s_average_sum = 0U;
    s_latest_raw = 0U;
    s_valid_mask = 0U;
    s_hw_error = 0U;

    /* 先使能 ADC DMA，避免 TIM2 触发到来时丢失首个转换结果。 */
    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)(void *)s_dma_buffer,
                          CURRENT_SENSE_DMA_BUFFER_SIZE) != HAL_OK) {
        s_hw_error = 1U;
        return CURRENT_SENSE_ERR_HW_FAILURE;
    }

    s_started = 1U;

    /* CH1 不输出到引脚，仅使用其 OC1REF 作为 ADC 外部触发源。 */
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        (void)HAL_ADC_Stop_DMA(&hadc1);
        s_started = 0U;
        s_hw_error = 1U;
        return CURRENT_SENSE_ERR_HW_FAILURE;
    }

    return CURRENT_SENSE_OK;
}

/* 停止 H1 电流采样并使当前缓冲数据失效。 */
CurrentSenseStatus_t CurrentSense_Stop(void)
{
    HAL_StatusTypeDef tim_status;
    HAL_StatusTypeDef adc_status;

    if (s_started == 0U) {
        return CURRENT_SENSE_ERR_NOT_STARTED;
    }

    /* 停止触发源后再停止 ADC DMA，避免关停过程中产生额外采样。 */
    tim_status = HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    adc_status = HAL_ADC_Stop_DMA(&hadc1);

    s_started = 0U;
    s_valid_mask = 0U;

    if ((tim_status != HAL_OK) || (adc_status != HAL_OK)) {
        s_hw_error = 1U;
        return CURRENT_SENSE_ERR_HW_FAILURE;
    }

    return CURRENT_SENSE_OK;
}

/* 获取最近一次安全完成的 ADC 原始样本。 */
CurrentSenseStatus_t CurrentSense_GetLatestRaw(uint16_t *raw)
{
    if (raw == NULL) {
        return CURRENT_SENSE_ERR_NULL_PTR;
    }
    if (s_hw_error != 0U) {
        return CURRENT_SENSE_ERR_HW_FAILURE;
    }
    if (s_started == 0U) {
        return CURRENT_SENSE_ERR_NOT_STARTED;
    }
    if (s_valid_mask == 0U) {
        return CURRENT_SENSE_ERR_NOT_READY;
    }

    *raw = s_latest_raw;
    return CURRENT_SENSE_OK;
}

/* 获取最近 32 个 PWM 同步采样点的平均 ADC 码值。 */
CurrentSenseStatus_t CurrentSense_GetAverageRaw(uint16_t *raw)
{
    uint32_t sum;

    if (raw == NULL) {
        return CURRENT_SENSE_ERR_NULL_PTR;
    }
    if (s_hw_error != 0U) {
        return CURRENT_SENSE_ERR_HW_FAILURE;
    }
    if (s_started == 0U) {
        return CURRENT_SENSE_ERR_NOT_STARTED;
    }
    if (s_valid_mask != CURRENT_SENSE_ALL_VALID) {
        return CURRENT_SENSE_ERR_NOT_READY;
    }

    sum = s_average_sum;
    *raw = (uint16_t)((sum + (CURRENT_SENSE_DMA_BUFFER_SIZE / 2U)) /
                      CURRENT_SENSE_DMA_BUFFER_SIZE);
    return CURRENT_SENSE_OK;
}

/* 保存零电流偏置和每个 ADC 码值对应的安培数。 */
CurrentSenseStatus_t CurrentSense_SetCalibration(float offset_counts,
                                                 float amps_per_count)
{
    if ((offset_counts < 0.0f) || (offset_counts > 4095.0f) ||
        (amps_per_count == 0.0f) ||
        (offset_counts != offset_counts) ||
        (amps_per_count != amps_per_count)) {
        return CURRENT_SENSE_ERR_INVALID_CALIBRATION;
    }

    s_offset_counts = offset_counts;
    s_amps_per_count = amps_per_count;
    s_calibrated = 1U;
    return CURRENT_SENSE_OK;
}

/* 使用已设置的标定参数，将 32 点平均 ADC 值换算为安培。 */
CurrentSenseStatus_t CurrentSense_GetCurrent(float *current_a)
{
    uint32_t sum;
    float average_counts;

    if (current_a == NULL) {
        return CURRENT_SENSE_ERR_NULL_PTR;
    }
    if (s_hw_error != 0U) {
        return CURRENT_SENSE_ERR_HW_FAILURE;
    }
    if (s_started == 0U) {
        return CURRENT_SENSE_ERR_NOT_STARTED;
    }
    if (s_valid_mask != CURRENT_SENSE_ALL_VALID) {
        return CURRENT_SENSE_ERR_NOT_READY;
    }
    if (s_calibrated == 0U) {
        return CURRENT_SENSE_ERR_NOT_CALIBRATED;
    }

    sum = s_average_sum;
    average_counts = (float)sum / (float)CURRENT_SENSE_DMA_BUFFER_SIZE;
    *current_a = (average_counts - s_offset_counts) * s_amps_per_count;
    return CURRENT_SENSE_OK;
}

/* HAL DMA 半传输回调：处理前半区。 */
void CurrentSense_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* 前 16 点写满后，DMA 已转向后半区，前半区可安全处理。 */
    if ((hadc != NULL) && (hadc->Instance == ADC1) && (s_started != 0U)) {
        CurrentSense_PublishHalf(0U);
    }
}

/* HAL DMA 全传输回调：处理后半区。 */
void CurrentSense_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* 后 16 点写满后，DMA 已回到前半区，后半区可安全处理。 */
    if ((hadc != NULL) && (hadc->Instance == ADC1) && (s_started != 0U)) {
        CurrentSense_PublishHalf(1U);
    }
}

/* HAL ADC/DMA 错误回调：标记故障并使当前数据失效。 */
void CurrentSense_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    /* 错误后废弃已有采样结果，等待调用方停止并重新启动模块。 */
    if ((hadc != NULL) && (hadc->Instance == ADC1)) {
        s_hw_error = 1U;
        s_valid_mask = 0U;
    }
}
