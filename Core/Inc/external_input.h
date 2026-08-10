#ifndef __EXTERNAL_INPUT_H
#define __EXTERNAL_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    EXTERNAL_INPUT_HALF_LOCK = 0, /* PF1  / HALF */
    EXTERNAL_INPUT_FULL_LOCK,     /* PF2  / FULL */
    EXTERNAL_INPUT_CENTRAL_LOCK,  /* PF3  / CENTRAL */
    EXTERNAL_INPUT_KEY1,          /* PF13 / KEY1 */
    EXTERNAL_INPUT_KEY2,          /* PF14 / KEY2 */
    EXTERNAL_INPUT_KEY3,          /* PF15 / KEY3 */
    EXTERNAL_INPUT_COUNT
} ExternalInputId_t;

typedef uint32_t ExternalInputMask_t;

/* 每个输入 ID 固定对应事件/状态位图中的同序 bit。 */
#define EXTERNAL_INPUT_MASK(id) \
    ((ExternalInputMask_t)1UL << (uint32_t)(id))
#define EXTERNAL_INPUT_ALL_MASK \
    ((ExternalInputMask_t)((1UL << (uint32_t)EXTERNAL_INPUT_COUNT) - 1UL))

typedef enum {
    EXTERNAL_INPUT_OK = 0,
    EXTERNAL_INPUT_ERR_NULL_PTR = -1,
    EXTERNAL_INPUT_ERR_INVALID_PARAM = -2,
    EXTERNAL_INPUT_ERR_NOT_INITIALIZED = -3,
    EXTERNAL_INPUT_ERR_ALREADY_INITIALIZED = -4
} ExternalInputStatus_t;

/*
 * 读取当前 GPIO 电平作为初始稳定状态，不生成上电边沿。
 * 必须在 GPIO 初始化完成、周期采样尚未启动时调用。
 */
ExternalInputStatus_t ExternalInput_Init(void);

/*
 * 单生产者周期调用。elapsed_ms 是距上一次采样的毫秒数：裸机 TIM6 传 1，
 * FreeRTOS 传感器任务可按 5 ms 周期传 5。此接口不执行任何业务指令。
 */
ExternalInputStatus_t ExternalInput_Service(uint32_t elapsed_ms);

/* 读取去抖后的电气电平；is_high 为 1 表示 HIGH。 */
ExternalInputStatus_t ExternalInput_GetLevel(ExternalInputId_t id,
                                             uint8_t *is_high);

/* 六路均为低有效；is_active 为 1 表示开关闭合或按键按下。 */
ExternalInputStatus_t ExternalInput_IsActive(ExternalInputId_t id,
                                             uint8_t *is_active);

/*
 * 单消费者接口。返回并消费自上次调用后出现过的去抖边沿：
 * rising_mask 表示 LOW->HIGH，falling_mask 表示 HIGH->LOW。
 * 六路均为低有效，因此 falling 通常表示激活，rising 通常表示释放。
 * 同一方向的重复事件折叠为一个 bit，不表示次数或先后顺序。
 */
ExternalInputStatus_t ExternalInput_TakeEdges(
    ExternalInputMask_t *rising_mask,
    ExternalInputMask_t *falling_mask);

/* 只检查是否存在未消费边沿，不改变消费者游标。 */
ExternalInputStatus_t ExternalInput_HasPendingEdges(uint8_t *has_event);

#ifdef __cplusplus
}
#endif

#endif /* __EXTERNAL_INPUT_H */
