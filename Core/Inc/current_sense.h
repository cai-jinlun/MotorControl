#ifndef __CURRENT_SENSE_H
#define __CURRENT_SENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define CURRENT_SENSE_DMA_BUFFER_SIZE  32U
#define CURRENT_SENSE_DMA_HALF_SIZE    (CURRENT_SENSE_DMA_BUFFER_SIZE / 2U)

typedef enum {
    CURRENT_SENSE_OK = 0,
    CURRENT_SENSE_ERR_NULL_PTR = -1,
    CURRENT_SENSE_ERR_NOT_STARTED = -2,
    CURRENT_SENSE_ERR_NOT_READY = -3,
    CURRENT_SENSE_ERR_NOT_CALIBRATED = -4,
    CURRENT_SENSE_ERR_INVALID_CALIBRATION = -5,
    CURRENT_SENSE_ERR_HW_FAILURE = -6
} CurrentSenseStatus_t;

/*
 * Start/stop H1 current acquisition.
 * ADC1 is triggered by TIM2_CH1 TRGO and writes into a circular DMA buffer.
 */
CurrentSenseStatus_t CurrentSense_Start(void);
CurrentSenseStatus_t CurrentSense_Stop(void);

/*
 * LatestRaw is the last sample from the most recently completed DMA half.
 * AverageRaw is the rounded mean of the latest two completed 16-sample halves.
 */
CurrentSenseStatus_t CurrentSense_GetLatestRaw(uint16_t *raw);
CurrentSenseStatus_t CurrentSense_GetAverageRaw(uint16_t *raw);

/* current[A] = (average_adc_counts - offset_counts) * amps_per_count */
CurrentSenseStatus_t CurrentSense_SetCalibration(float offset_counts,
                                                 float amps_per_count);
CurrentSenseStatus_t CurrentSense_GetCurrent(float *current_a);

/* HAL callback forwarding functions. */
void CurrentSense_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void CurrentSense_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void CurrentSense_ErrorCallback(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif /* __CURRENT_SENSE_H */
