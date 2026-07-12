#ifndef __MOTOR_BDC_VNH_H
#define __MOTOR_BDC_VNH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_driver.h"
#include "stm32f4xx_hal.h"   /* 根据你的HAL库路径调整 */

/* =====================================================================
 * MOTOR_BDC_VNH配置参数（用户可见，仅含硬件连接信息）
 * ===================================================================== */
typedef struct {
    TIM_HandleTypeDef *htim;          /* PWM定时器句柄 */
    uint32_t           tim_channel;    /* PWM通道 */
    GPIO_TypeDef      *gpio_port_a;    /* 方向控制脚A */
    uint16_t           gpio_pin_a;
    GPIO_TypeDef      *gpio_port_b;    /* 方向控制脚B */
    uint16_t           gpio_pin_b;
    uint32_t           pwm_full_scale; /* 对应100%占空比的ARR比较值（如1000） */
    uint16_t           dead_zone;      /* 死区：|speed| < dead_zone 时强制输出0 */
} MotorBDC_VNH_Config_t;

/* =====================================================================
 * 模块与工厂函数
 * ===================================================================== */

/* 注册直流电机操作表到核心调度层（系统启动时调用一次） */
void MotorBDC_VNH_ModuleInit(void);

/* 工厂函数：创建直流电机实例，返回公共句柄指针（失败返回NULL） */
MotorHandle_t* MotorBDC_VNH_Create(const MotorBDC_VNH_Config_t *cfg);

/* 销毁实例，回收内存池与硬件资源 */
void MotorBDC_VNH_Destroy(MotorHandle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_BDC_VNH_H */