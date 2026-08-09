#ifndef __MOTOR_BDC_DRV_H
#define __MOTOR_BDC_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_driver.h"
#include <stdint.h>

/*
 * motor_bdc_drv 与具体 MCU/板卡之间的连接接口。
 *
 * 生命周期约定：
 * 1. 每个成功创建的电机实例只调用一次 init。
 * 2. 每次有效销毁只调用一次 deinit；板级层可在其中实现引用计数。
 * 3. init 返回失败时，必须由 init 自行回滚已启动的板级资源；驱动层
 *    不会再调用 deinit 拆解这些资源。
 * 4. init 成功后若驱动的后续初始化失败，驱动会调用一次 deinit，以平衡
 *    已成功的 init。
 * 5. deinit 即使返回失败，驱动也不会再次调用它；板级 deinit 应在单次调用
 *    内完成引用计数扣减和必要的错误收敛。
 */
typedef struct {
    /* 初始化本电机实例所需的板级资源，可在内部执行引用计数加一。 */
    MotorErr_t (*init)(void *context);
    /* 释放本电机实例所占用的板级资源，可在内部执行引用计数减一。 */
    MotorErr_t (*deinit)(void *context);

    /*
     * 一次提交 IN1/IN2，数值范围均为 0~MOTOR_OUTPUT_MAX。
     * 板级层负责换算为 CCR，并尽可能同步更新两个 PWM 通道。
     */
    MotorErr_t (*set_inputs)(void *context, uint16_t in1, uint16_t in2);

    /* 读取霍尔/编码器原始位置计数。 */
    MotorErr_t (*get_position)(void *context, int32_t *position);
    /* 将霍尔/编码器位置计数设置为指定值。 */
    MotorErr_t (*reset_position)(void *context, int32_t position);
    /* 读取带符号速度，正负号分别表示正转和反转。 */
    MotorErr_t (*get_velocity)(void *context, float *velocity);
    /* 读取已由板级层完成标定换算的电流值（单位：A）。 */
    MotorErr_t (*get_current)(void *context, float *current);
} MotorBDC_DRV_PortOps_t;

typedef struct {
    /* 板级端口操作表；其生命周期必须长于电机实例。 */
    const MotorBDC_DRV_PortOps_t *port;
    /* 原样传递给端口操作表的用户上下文。 */
    void                         *context;
    /* 小于该值的正反转输出会被置零，范围为 0~1000。 */
    uint16_t                      dead_zone;
} MotorBDC_DRV_Config_t;

/*
 * 创建会自动完成 Motor_Init；失败返回 NULL。配置内容会被复制到实例中。
 * error 可为 NULL；非 NULL 时返回成功状态或具体失败原因。
 */
MotorHandle_t *MotorBDC_DRV_Create(const MotorBDC_DRV_Config_t *cfg,
                                   MotorErr_t *error);

/* 仅销毁由 MotorBDC_DRV_Create 返回且尚未销毁的有效实例。 */
void MotorBDC_DRV_Destroy(MotorHandle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_BDC_DRV_H */
