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
    MotorErr_t (*init)(void *context);
    MotorErr_t (*deinit)(void *context);

    /*
     * 一次提交 IN1/IN2，数值范围均为 0~MOTOR_OUTPUT_MAX。
     * 板级层负责换算为 CCR，并尽可能同步更新两个 PWM 通道。
     */
    MotorErr_t (*set_inputs)(void *context, uint16_t in1, uint16_t in2);

    MotorErr_t (*get_position)(void *context, int32_t *position);
    MotorErr_t (*reset_position)(void *context, int32_t position);
    MotorErr_t (*get_velocity)(void *context, float *velocity);
    MotorErr_t (*get_current)(void *context, float *current);
} MotorBDC_DRV_PortOps_t;

typedef struct {
    const MotorBDC_DRV_PortOps_t *port;
    void                         *context;
    uint16_t                      dead_zone;
} MotorBDC_DRV_Config_t;

/* 将 MOTOR_TYPE_BDC_DRV 操作表注册到通用电机层；系统启动时调用一次。 */
MotorErr_t MotorBDC_DRV_ModuleInit(void);

/* 创建会自动完成 Motor_Init；失败返回 NULL。配置内容会被复制到实例中。 */
MotorHandle_t *MotorBDC_DRV_Create(const MotorBDC_DRV_Config_t *cfg);

/* 仅销毁由 MotorBDC_DRV_Create 返回且尚未销毁的有效实例。 */
void MotorBDC_DRV_Destroy(MotorHandle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_BDC_DRV_H */
