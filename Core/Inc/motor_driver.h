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

static inline int16_t Motor_Clamp(int16_t value,
                                  int16_t min_value,
                                  int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

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
    MOTOR_ERR_NOT_READY = -8,         /* 测量数据尚未准备完成 */
    MOTOR_ERR_TIMEOUT_LATCHED = -9,   /* 运行超时已锁存，拒绝再次运行 */
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

typedef uint32_t (*MotorTimeSource_t)(void);

/*
 * 当前一次连续运行的监控状态。FORWARD/BACKWARD 之间直接换向不会重置
 * start_time_ms；只有成功进入 COAST/BRAKE 才结束本次计时。
 */
typedef struct {
    uint32_t         start_time_ms;     /* 从停止态进入运行态时的毫秒时间戳 */
    uint32_t         timeout_ms;        /* 0 表示禁用自动超时保护 */
    MotorDirection_t timeout_stop_mode; /* 超时后只允许 COAST 或 BRAKE */
    uint8_t          is_running;        /* 缓存的实际驱动状态是否为正/反转 */
    uint8_t          timeout_latched;   /* 锁存后必须由上层显式清除 */
} MotorRunMonitor_t;

typedef struct {
    MotorErr_t (*init)(struct MotorHandle *motor);
    MotorErr_t (*deinit)(struct MotorHandle *motor);
    /* 以下输出回调的 output 已由通用层限制在 MOTOR_OUTPUT_MIN..MOTOR_OUTPUT_MAX。 */
    MotorErr_t (*setOutput)(struct MotorHandle *motor, int16_t output);
    MotorErr_t (*setDirOutput)(struct MotorHandle *motor, MotorDirection_t dir, int16_t output);
    MotorErr_t (*resetPosition)(struct MotorHandle *motor, int32_t position);  /* 可选：位置设置，主要为重置位置 */
    // MotorErr_t (*getStatus)(const struct MotorHandle *motor);

    MotorErr_t (*getOutput)(const struct MotorHandle *motor, int16_t *output);  /* 可选：驱动输出读取 */
    MotorErr_t (*getDriveDirection)(const struct MotorHandle *motor, MotorDirection_t *dir); /* 必选：用于同步运行计时状态 */
    MotorErr_t (*getMeasuredDirection)(const struct MotorHandle *motor, MotorDirection_t *dir); /* 可选：测量方向读取 */
    MotorErr_t (*getMeasuredPosition)(const struct MotorHandle *motor, int32_t *position); /* 可选：测量位置读取 */
    MotorErr_t (*getMeasuredVelocity)(const struct MotorHandle *motor, float *velocity); /* 可选：测量速度读取 */
    MotorErr_t (*getMeasuredCurrent)(const struct MotorHandle *motor, float *current); /* 可选：测量电流读取 */
} MotorOps_t;

/* =====================================================================
 * 电机句柄（基类）
 * ===================================================================== */
typedef struct MotorHandle {
    MotorType_t       type;            /* 电机类型 */
    uint8_t           is_initialized;  /* 初始化标志 */
    const MotorOps_t *ops;             /* 操作表（只读） */
    void             *priv;            /* 私有数据（外部严禁访问） */
    MotorRunMonitor_t run_monitor;      /* 通用运行计时与超时状态 */
} MotorHandle_t;

/* =====================================================================
 * 统一API（上层业务只调用这些）
 * ===================================================================== */
MotorErr_t Motor_Init(MotorHandle_t *motor);
MotorErr_t Motor_Deinit(MotorHandle_t *motor);

MotorErr_t Motor_SetOutput(MotorHandle_t *motor, int16_t output);
MotorErr_t Motor_SetDirOutput(MotorHandle_t *motor, MotorDirection_t dir, int16_t output);
MotorErr_t Motor_ResetPosition(MotorHandle_t *motor, int32_t position);
/* 在任何电机进入运行状态前配置；运行期间不得切换时间源。 */
MotorErr_t Motor_SetTimeSource(MotorTimeSource_t time_source);
/* 修改阈值不会清除已有超时锁存；stop_mode 仅允许 COAST 或 BRAKE。 */
MotorErr_t Motor_ConfigureRunTimeout(MotorHandle_t *motor,
                                     uint32_t timeout_ms,
                                     MotorDirection_t stop_mode);
/* 返回当前一次连续运行时间；停止状态固定返回 0。 */
MotorErr_t Motor_GetRunningTime(const MotorHandle_t *motor, uint32_t *time_ms);
MotorErr_t Motor_GetRunTimeout(const MotorHandle_t *motor, uint8_t *timed_out);
/* 仅允许在实际驱动方向已经停止时清除超时锁存。 */
MotorErr_t Motor_ClearRunTimeout(MotorHandle_t *motor);
/*
 * 周期执行超时检查。裸机由主循环调用；FreeRTOS 下应由唯一的电机控制
 * 任务串行调用，禁止从 ISR 或多个任务并发调用电机控制接口。
 */
MotorErr_t Motor_Service(MotorHandle_t *motor);
    
MotorErr_t Motor_GetOutput(const MotorHandle_t *motor, int16_t *output);
MotorErr_t Motor_GetDriveDirection(const MotorHandle_t *motor, MotorDirection_t *dir);
MotorErr_t Motor_GetMeasuredDirection(const MotorHandle_t *motor, MotorDirection_t *dir);
MotorErr_t Motor_GetMeasuredPosition(const MotorHandle_t *motor, int32_t *position);
MotorErr_t Motor_GetMeasuredVelocity(const MotorHandle_t *motor, float *velocity);
MotorErr_t Motor_GetMeasuredCurrent(const MotorHandle_t *motor, float *current);

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
