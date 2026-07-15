#ifndef __CURRENT_SENSE_H
#define __CURRENT_SENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define CURRENT_SENSE_DMA_BUFFER_SIZE  32U
#define CURRENT_SENSE_DMA_HALF_SIZE    (CURRENT_SENSE_DMA_BUFFER_SIZE / 2U)

/* 电流采样模块接口返回状态。 */
typedef enum {
    CURRENT_SENSE_OK = 0,                    /* 操作成功 */
    CURRENT_SENSE_ERR_NULL_PTR = -1,         /* 输出参数为空 */
    CURRENT_SENSE_ERR_NOT_STARTED = -2,      /* 采样尚未启动 */
    CURRENT_SENSE_ERR_NOT_READY = -3,        /* DMA 尚未采满有效数据 */
    CURRENT_SENSE_ERR_NOT_CALIBRATED = -4,   /* 尚未设置电流换算参数 */
    CURRENT_SENSE_ERR_INVALID_CALIBRATION = -5, /* 标定参数非法 */
    CURRENT_SENSE_ERR_HW_FAILURE = -6        /* ADC、DMA 或定时器操作失败 */
} CurrentSenseStatus_t;

/*
 * 启动或停止 H1 电流采样。
 * ADC1 由 TIM2_CH1 的 TRGO 触发，转换结果写入 DMA 环形缓冲区。
 */
CurrentSenseStatus_t CurrentSense_Start(void);
CurrentSenseStatus_t CurrentSense_Stop(void);

/*
 * 获取原始 ADC 数据。
 * LatestRaw 为最近完成 DMA 半区的最后一个样本；AverageRaw 为最近两个
 * 16 点半区组成的 32 点平均值，并按四舍五入转换为整数。
 */
CurrentSenseStatus_t CurrentSense_GetLatestRaw(uint16_t *raw);
CurrentSenseStatus_t CurrentSense_GetAverageRaw(uint16_t *raw);

/*
 * 设置电流换算标定参数。
 * 电流值[A] = (平均 ADC 码值 - offset_counts) * amps_per_count。
 */
CurrentSenseStatus_t CurrentSense_SetCalibration(float offset_counts,
                                                 float amps_per_count);

/* 获取经标定换算后的平均电流，单位为安培。 */
CurrentSenseStatus_t CurrentSense_GetCurrent(float *current_a);

/* 由 HAL ADC 半传输、全传输和错误回调转发调用。 */
void CurrentSense_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void CurrentSense_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void CurrentSense_ErrorCallback(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif /* 电流采样模块头文件 */
