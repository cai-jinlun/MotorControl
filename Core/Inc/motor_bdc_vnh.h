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
 * 硬件前提：
 * 本类电机（门锁解锁/复位电机与吸合电机）在机构上没有霍尔传感器，
 * 控制板也未为其提供 ADC 电流采集，因此本驱动不提供位置、速度与
 * 电流反馈能力；对应的通用电机 API 将返回 MOTOR_ERR_NOT_SUPPORTED。
 * 此类电机的行程控制与超时保护以运行时间（Motor_SetRunningTime /
 * Motor_GetRunningTime）作为主要判据。
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
    /* 初始化本电机实例所需的板级资源，可在内部执行引用计数加一。 */
    MotorErr_t (*init)(void *context);
    /* 释放本电机实例所占用的板级资源，可在内部执行引用计数减一。 */
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

    /*
     * 获取系统毫秒时钟（通常绑定 HAL_GetTick），用于运行时间统计。
     * 板级层负责提供；驱动层不直接依赖 HAL 时钟。
     */
    MotorErr_t (*get_time_ms)(void *context, uint32_t *time_ms);
} MotorBDC_VNH_PortOps_t;

typedef struct {
    /* 板级端口操作表；其生命周期必须长于电机实例。 */
    const MotorBDC_VNH_PortOps_t *port;
    /* 原样传递给端口操作表的用户上下文。 */
    void                         *context;
    /* 小于该值的正反转输出会被置零，范围为 0~1000。 */
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
