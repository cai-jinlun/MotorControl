#include "motor_driver.h"
#include <string.h>

/* 操作表注册中心 */
static const MotorOps_t *g_ops_registry[MOTOR_TYPE_MAX] = {NULL};

/* ------------------------------------------------------------------ */
MotorErr_t Motor_RegisterOps(MotorType_t type, const MotorOps_t *ops)
{
    if(ops == NULL) return MOTOR_ERR_NULL_PTR;
    if (type >= MOTOR_TYPE_MAX ) {
        return MOTOR_ERR_INVALID_PARAM;
    }
    if (g_ops_registry[type] != NULL) {
        return MOTOR_ERR_ALREADY_INIT;   /* 防止重复注册 */
    }
    g_ops_registry[type] = ops;
    return MOTOR_OK;
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_Init(MotorHandle_t *motor)
{
    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (motor->is_initialized) return MOTOR_ERR_ALREADY_INIT;
    if (motor->ops == NULL || motor->ops->init == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    MotorErr_t err = motor->ops->init(motor);
    if (err == MOTOR_OK) {
        motor->is_initialized = 1;
    }
    return err;
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_Deinit(MotorHandle_t *motor)
{
    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->deinit == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    MotorErr_t err = motor->ops->deinit(motor);
    if (err == MOTOR_OK) {
        motor->is_initialized = 0;
    }
    return err;
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_SetSpeed(MotorHandle_t *motor, int16_t speed)
{
    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->setSpeed == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }
    speed = CLAMP(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

    return motor->ops->setSpeed(motor, speed);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_SetDirSpeed(MotorHandle_t *motor, MotorDirection_t dir, int16_t speed)
{
    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->setDirSpeed == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }
    if (dir >= MOTOR_DIR_MAX) {
        return MOTOR_ERR_INVALID_PARAM;
    }
    speed = CLAMP(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

    return motor->ops->setDirSpeed(motor, dir, speed);
}