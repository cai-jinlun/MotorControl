#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


/* =====================================================================
 * 输出量程（所有电机统一映射到 0~1000）
 * ===================================================================== */
#define MOTOR_OUTPUT_MAX  1000
#define MOTOR_OUTPUT_MIN  0
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* =====================================================================
 * 错误码
 * ===================================================================== */
typedef enum {
    MOTOR_OK = 0,
    MOTOR_ERR_NULL_PTR = -1,          /* 空指针 */
    MOTOR_ERR_INVALID_PARAM = -2,     /* 参数越界 */
    MOTOR_ERR_NOT_INITIALIZED = -3,   /* 句柄未初始化 */
    MOTOR_ERR_NO_RESOURCE = -4,       /* 静态池耗尽 */
    MOTOR_ERR_HW_FAILURE = -5,        /* HAL/硬件错误 */
    MOTOR_ERR_NOT_SUPPORTED = -6,     /* 功能未实现 */
    MOTOR_ERR_ALREADY_INIT = -7,      /* 重复初始化 */
} MotorErr_t;

/* =====================================================================
 * 方向定义
 * ===================================================================== */
typedef enum {
    MOTOR_DIR_FORWARD = 0,   /* 正转 */
    MOTOR_DIR_BACKWARD,      /* 反转 */
    MOTOR_DIR_COAST,         /* 悬停（自由滑行） */
    MOTOR_DIR_BRAKE,         /* 刹车（主动制动） */
    MOTOR_DIR_MAX
} MotorDirection_t;

/* =====================================================================
 * 电机类型_驱动芯片/驱动方式
 * 未来可扩展更多类型（比如纹波电机、无刷电机等）
 * ===================================================================== */
typedef enum {
    MOTOR_TYPE_BDC_VNH = 0, /* 直流有刷电机，使用VNH驱动芯片 */
    MOTOR_TYPE_BDC_DRV,     /* 直流有刷电机，使用DRV驱动芯片 */
    MOTOR_TYPE_MAX
} MotorType_t;

/* =====================================================================
 * 操作表（函数指针表）
 * ===================================================================== */
struct MotorHandle;   /* 前向声明 */

typedef struct {
    MotorErr_t (*init)(struct MotorHandle *motor);
    MotorErr_t (*deinit)(struct MotorHandle *motor);
    MotorErr_t (*setOutput)(struct MotorHandle *motor, int16_t output);
    MotorErr_t (*setDirOutput)(struct MotorHandle *motor, MotorDirection_t dir, int16_t output);
    MotorErr_t (*resetPosition)(struct MotorHandle *motor, int32_t position);  /* 可选：位置设置，主要为重置位置 */
    MotorErr_t (*setRunningTime)(struct MotorHandle *motor, uint32_t time_ms); /* 可选：设置运行时间（ms） */
    // MotorErr_t (*getStatus)(const struct MotorHandle *motor);

    MotorErr_t (*getOutput)(const struct MotorHandle *motor, int16_t *output);  /* 可选：驱动输出读取 */
    MotorErr_t (*getDirection)(const struct MotorHandle *motor, MotorDirection_t *dir);  /* 可选：驱动方向读取 */
    MotorErr_t (*getPosition)(const struct MotorHandle *motor, int32_t *position);  /* 可选：位置读取 */
    MotorErr_t (*getVelocity)(const struct MotorHandle *motor, float *velocity);  /* 可选：速度（其他方式获得的）读取 */
    MotorErr_t (*getCurrent)(const struct MotorHandle *motor, float *current);  /* 可选：电流读取 */
    MotorErr_t (*getRunningTime)(const struct MotorHandle *motor, uint32_t *time_ms); /* 可选：运行时间读取（ms） */
} MotorOps_t;

/* =====================================================================
 * 电机句柄（基类）
 * ===================================================================== */
typedef struct MotorHandle {
    uint8_t           id;              /* 实例编号 */
    MotorType_t       type;            /* 电机类型 */
    uint8_t           is_initialized;  /* 初始化标志 */
    const MotorOps_t *ops;             /* 操作表（只读） */
    void             *priv;            /* 私有数据（外部严禁访问） */
} MotorHandle_t;

/* =====================================================================
 * 统一API（上层业务只调用这些）
 * ===================================================================== */
MotorErr_t Motor_RegisterOps(MotorType_t type, const MotorOps_t *ops);

MotorErr_t Motor_Init(MotorHandle_t *motor);
MotorErr_t Motor_Deinit(MotorHandle_t *motor);

MotorErr_t Motor_SetOutput(MotorHandle_t *motor, int16_t output);
MotorErr_t Motor_SetDirOutput(MotorHandle_t *motor, MotorDirection_t dir, int16_t output);
MotorErr_t Motor_ResetPosition(MotorHandle_t *motor, int32_t position);
MotorErr_t Motor_SetRunningTime(MotorHandle_t *motor, uint32_t time_ms);
    
MotorErr_t Motor_GetOutput(const MotorHandle_t *motor, int16_t *output);
MotorErr_t Motor_GetDirection(const MotorHandle_t *motor, MotorDirection_t *dir);
MotorErr_t Motor_GetPosition(const MotorHandle_t *motor, int32_t *position);
MotorErr_t Motor_GetVelocity(const MotorHandle_t *motor, float *velocity);
MotorErr_t Motor_GetCurrent(const MotorHandle_t *motor, float *current);
MotorErr_t Motor_GetRunningTime(const MotorHandle_t *motor, uint32_t *time_ms);

/* 便捷函数：停止 */
// static inline MotorErr_t Motor_Stop(MotorHandle_t *motor, MotorDirection_t mode)
// {
//     if (mode != MOTOR_DIR_COAST && mode != MOTOR_DIR_BRAKE) {
//         return MOTOR_ERR_INVALID_PARAM;
//     }
// }

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H */
