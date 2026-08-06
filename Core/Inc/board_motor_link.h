#ifndef __BOARD_MOTOR_LINK_H
#define __BOARD_MOTOR_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_driver.h"

/**
 * @brief 初始化开发板上的三个业务电机及其底层资源。
 *
 * 必须在 CubeMX 生成的 GPIO、DMA、ADC、SPI 和 TIM 初始化完成后调用。
 * 初始化成功时三个电机均处于 MOTOR_DIR_COAST、零输出状态。
 */
MotorErr_t BoardMotorLink_Init(void);

/* Getter 返回的句柄由板级链接层持有，上层仅通过 Motor_* API 使用。 */
MotorHandle_t *BoardMotorLink_GetDoorMotor(void);   /* 车门撑杆电机：DRV8714 / H1 */
MotorHandle_t *BoardMotorLink_GetUnlockMotor(void); /* 解锁电机：VNH7070 / H4 */
MotorHandle_t *BoardMotorLink_GetCinchMotor(void);  /* 吸合电机：VNH7070 / H5 */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_MOTOR_LINK_H */
