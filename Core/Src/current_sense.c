#include "current_sense.h"
#include "adc.h"
#include "tim.h"

#define CURRENT_SENSE_HALF_0_VALID  (1U << 0)
#define CURRENT_SENSE_HALF_1_VALID  (1U << 1)
#define CURRENT_SENSE_ALL_VALID     (CURRENT_SENSE_HALF_0_VALID | \
                                     CURRENT_SENSE_HALF_1_VALID)

static volatile uint16_t s_dma_buffer[CURRENT_SENSE_DMA_BUFFER_SIZE];
static volatile uint32_t s_half_sum[2];
static volatile uint32_t s_average_sum;
static volatile uint16_t s_latest_raw;
static volatile uint8_t s_valid_mask;
static volatile uint8_t s_started;
static volatile uint8_t s_hw_error;

static float s_offset_counts;
static float s_amps_per_count;
static uint8_t s_calibrated;

static uint32_t CurrentSense_SumHalf(uint32_t start)
{
    uint32_t sum = 0U;
    uint32_t i;

    for (i = start; i < (start + CURRENT_SENSE_DMA_HALF_SIZE); ++i) {
        sum += s_dma_buffer[i];
    }
    return sum;
}

static void CurrentSense_PublishHalf(uint32_t half)
{
    uint32_t start = half * CURRENT_SENSE_DMA_HALF_SIZE;
    uint32_t sum = CurrentSense_SumHalf(start);
    uint8_t valid_bit = (half == 0U) ? CURRENT_SENSE_HALF_0_VALID
                                     : CURRENT_SENSE_HALF_1_VALID;
    uint8_t new_valid_mask = s_valid_mask | valid_bit;

    s_half_sum[half] = sum;
    s_latest_raw = s_dma_buffer[start + CURRENT_SENSE_DMA_HALF_SIZE - 1U];

    /* Publish the ready flag only after the first valid average is stored. */
    if (new_valid_mask == CURRENT_SENSE_ALL_VALID) {
        s_average_sum = s_half_sum[0] + s_half_sum[1];
    }
    s_valid_mask = new_valid_mask;
}

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

    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)(void *)s_dma_buffer,
                          CURRENT_SENSE_DMA_BUFFER_SIZE) != HAL_OK) {
        s_hw_error = 1U;
        return CURRENT_SENSE_ERR_HW_FAILURE;
    }

    s_started = 1U;

    /* CH1 has no external pin; its OC1REF is the ADC trigger source. */
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        (void)HAL_ADC_Stop_DMA(&hadc1);
        s_started = 0U;
        s_hw_error = 1U;
        return CURRENT_SENSE_ERR_HW_FAILURE;
    }

    return CURRENT_SENSE_OK;
}

CurrentSenseStatus_t CurrentSense_Stop(void)
{
    HAL_StatusTypeDef tim_status;
    HAL_StatusTypeDef adc_status;

    if (s_started == 0U) {
        return CURRENT_SENSE_ERR_NOT_STARTED;
    }

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

void CurrentSense_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) && (hadc->Instance == ADC1) && (s_started != 0U)) {
        CurrentSense_PublishHalf(0U);
    }
}

void CurrentSense_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) && (hadc->Instance == ADC1) && (s_started != 0U)) {
        CurrentSense_PublishHalf(1U);
    }
}

void CurrentSense_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) && (hadc->Instance == ADC1)) {
        s_hw_error = 1U;
        s_valid_mask = 0U;
    }
}
