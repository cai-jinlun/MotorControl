#ifndef __MOTOR_RUNTIME_H
#define __MOTOR_RUNTIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * motor_runtime：各电机类型驱动共用的运行时间统计内联实现。
 *
 * 设计约定：
 * 1. "运行"判据由类型驱动在死区处理之后给出（有效输出 > 0 且方向为
 *    FORWARD/BACKWARD），本模块只负责计时，不关心输出语义。
 * 2. 时间基准由板级 get_time_ms 回调提供（通常绑定 HAL_GetTick），
 *    本模块不直接依赖 HAL，便于将来切换到 FreeRTOS 时钟。
 * 3. 全部运算使用无符号回绕安全的差值计算，uint32 毫秒时钟回绕时
 *    结果仍然正确。
 * 4. 本模块为非重入设计：调用方（类型驱动）应保证同一电机实例的
 *    输出提交与运行时间访问不并发执行。
 */
typedef struct {
    uint32_t accumulated_ms;  /* 已结算的运行时间 */
    uint32_t start_ms;        /* 本次运行段的起始时刻 */
    uint8_t  is_running;      /* 当前是否处于有效输出状态 */
} MotorRuntime_t;

/*
 * 根据本次有效输出状态推进运行时间统计。
 * running 非 0 表示本次提交后电机处于有效驱动输出状态。
 */
static inline void MotorRuntime_Update(MotorRuntime_t *rt,
                                       uint8_t running,
                                       uint32_t now_ms)
{
    if (running != 0U) {
        if (rt->is_running == 0U) {
            rt->start_ms = now_ms;
            rt->is_running = 1U;
        }
    } else {
        if (rt->is_running != 0U) {
            rt->accumulated_ms += (now_ms - rt->start_ms);
            rt->is_running = 0U;
        }
    }
}

/*
 * 读取累计运行时间（ms）：已结算部分加上当前运行段的未结算部分。
 */
static inline uint32_t MotorRuntime_Get(const MotorRuntime_t *rt,
                                        uint32_t now_ms)
{
    uint32_t total = rt->accumulated_ms;

    if (rt->is_running != 0U) {
        total += (now_ms - rt->start_ms);
    }
    return total;
}

/*
 * 将累计运行时间设置为指定值。
 * 若当前处于运行段中，则将本段起始时刻重置为 now_ms，
 * 使后续读取从 time_ms 起继续累加。
 */
static inline void MotorRuntime_Set(MotorRuntime_t *rt,
                                    uint32_t time_ms,
                                    uint32_t now_ms)
{
    rt->accumulated_ms = time_ms;
    if (rt->is_running != 0U) {
        rt->start_ms = now_ms;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_RUNTIME_H */
