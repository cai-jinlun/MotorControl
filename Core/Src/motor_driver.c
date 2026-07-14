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
MotorErr_t Motor_SetOutput(MotorHandle_t *motor, int16_t output)
{
    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->setOutput == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }
    output = CLAMP(output, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);

    return motor->ops->setOutput(motor, output);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_SetDirOutput(MotorHandle_t *motor, MotorDirection_t dir, int16_t output)
{
    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->setDirOutput == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }
    if (dir >= MOTOR_DIR_MAX) {
        return MOTOR_ERR_INVALID_PARAM;
    }
    output = CLAMP(output, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);

    return motor->ops->setDirOutput(motor, dir, output);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_ResetPosition(MotorHandle_t *motor, int32_t position)
{
    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->resetPosition == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->resetPosition(motor, position);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_SetRunningTime(MotorHandle_t *motor, uint32_t time_ms)
{
    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->setRunningTime == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->setRunningTime(motor, time_ms);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_GetOutput(const MotorHandle_t *motor, int16_t *output)
{
    if (motor == NULL || output == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->getOutput == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getOutput(motor, output);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_GetDriveDirection(const MotorHandle_t *motor, MotorDirection_t *dir)
{
    if (motor == NULL || dir == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->getDriveDirection == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getDriveDirection(motor, dir);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_GetMeasuredDirection(const MotorHandle_t *motor, MotorDirection_t *dir)
{
    if (motor == NULL || dir == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->getMeasuredDirection == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getMeasuredDirection(motor, dir);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_GetMeasuredPosition(const MotorHandle_t *motor, int32_t *position)
{
    if (motor == NULL || position == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->getMeasuredPosition == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getMeasuredPosition(motor, position);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_GetMeasuredVelocity(const MotorHandle_t *motor, float *velocity)
{
    if (motor == NULL || velocity == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->getMeasuredVelocity == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getMeasuredVelocity(motor, velocity);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_GetMeasuredCurrent(const MotorHandle_t *motor, float *current)
{
    if (motor == NULL || current == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->getMeasuredCurrent == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getMeasuredCurrent(motor, current);
}

/* ------------------------------------------------------------------ */
MotorErr_t Motor_GetRunningTime(const MotorHandle_t *motor, uint32_t *time_ms)
{
    if (motor == NULL || time_ms == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->ops == NULL || motor->ops->getRunningTime == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getRunningTime(motor, time_ms);
}
