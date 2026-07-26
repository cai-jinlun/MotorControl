#ifndef __MOTOR_BDC_VNH_H
#define __MOTOR_BDC_VNH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_driver.h"
#include <stdint.h>

/*
 * motor_bdc_vnh 与具体 MCU/板卡之间的连接接口。
 *
 * 生命周期约定：
 * 1. 每个成功创建的实例只调用一次 init。
 * 2. 每次有效销毁只调用一次 deinit，板级层可在其中实现引用计数。
 * 3. init 返回失败时，由 init 自行回滚已启动的板级资源；驱动层不会
 *    再调用 deinit 拆解这些资源。
 * 4. init 成功后若驱动的后续初始化失败，驱动会调用一次 deinit，
 *    以平衡已经成功的 init。
 * 5. deinit 即使返回失败也不会被重复调用；板级 deinit 应在单次调用
 *    内完成引用计数扣减和必要的错误收敛。
 */
typedef struct {
    MotorErr_t (*init)(void *context);
    MotorErr_t (*deinit)(void *context);

    /*
     * 一次提交方向输入 A/B 与 PWM。
     * in_a、in_b 只允许为 0 或 1；pwm 范围为 0~MOTOR_OUTPUT_MAX。
     * 板级层负责 GPIO/TIM 映射以及 PWM 比较值换算。
     */
    MotorErr_t (*set_outputs)(void *context,
                              uint8_t in_a,
                              uint8_t in_b,
                              uint16_t pwm);

    /* 以下反馈能力可选；NULL 表示对应 MotorOps 不受支持。 */
    MotorErr_t (*get_position)(void *context, int32_t *position);
    MotorErr_t (*reset_position)(void *context, int32_t position);
    MotorErr_t (*get_velocity)(void *context, float *velocity);
    MotorErr_t (*get_current)(void *context, float *current);
} MotorBDC_VNH_PortOps_t;

typedef struct {
    const MotorBDC_VNH_PortOps_t *port;
    void                         *context;
    uint16_t                      dead_zone;
} MotorBDC_VNH_Config_t;

/* 将 MOTOR_TYPE_BDC_VNH 操作表注册到通用电机层；系统启动时调用一次。 */
MotorErr_t MotorBDC_VNH_ModuleInit(void);

/* 创建会复制配置并自动完成 Motor_Init；失败返回 NULL。 */
MotorHandle_t *MotorBDC_VNH_Create(const MotorBDC_VNH_Config_t *cfg);

/* 仅销毁由 MotorBDC_VNH_Create 返回且尚未销毁的有效实例。 */
void MotorBDC_VNH_Destroy(MotorHandle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_BDC_VNH_H */
