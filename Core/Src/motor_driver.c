#include "motor_driver.h"

static MotorTimeSource_t s_time_source;

static uint8_t motor_direction_is_running(MotorDirection_t direction)
{
    return ((direction == MOTOR_DIR_FORWARD) ||
            (direction == MOTOR_DIR_BACKWARD)) ? 1U : 0U;
}

/* Synchronize the generic timer with the direction actually cached by the driver. */
static MotorErr_t motor_sync_run_monitor(MotorHandle_t *motor)
{
    MotorDirection_t direction;
    MotorErr_t status;

    status = motor->ops->getDriveDirection(motor, &direction);
    if (status != MOTOR_OK) {
        return status;
    }

    if (motor_direction_is_running(direction) != 0U) {
        if (motor->run_monitor.is_running == 0U) {
            if (s_time_source == NULL) {
                return MOTOR_ERR_NOT_READY;
            }
            motor->run_monitor.start_time_ms = s_time_source();
            motor->run_monitor.is_running = 1U;
        }
    } else {
        motor->run_monitor.start_time_ms = 0U;
        motor->run_monitor.is_running = 0U;
    }

    return MOTOR_OK;
}

MotorErr_t Motor_Init(MotorHandle_t *motor)
{
    MotorErr_t status;

    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (motor->is_initialized) return MOTOR_ERR_ALREADY_INIT;
    if ((motor->ops == NULL) || (motor->ops->init == NULL) ||
        (motor->ops->getDriveDirection == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    status = motor->ops->init(motor);
    if (status == MOTOR_OK) {
        motor->run_monitor.start_time_ms = 0U;
        motor->run_monitor.is_running = 0U;
        motor->run_monitor.timeout_latched = 0U;
        if ((motor->run_monitor.timeout_stop_mode != MOTOR_DIR_COAST) &&
            (motor->run_monitor.timeout_stop_mode != MOTOR_DIR_BRAKE)) {
            motor->run_monitor.timeout_stop_mode = MOTOR_DIR_COAST;
        }
        motor->is_initialized = 1U;
    }
    return status;
}

MotorErr_t Motor_Deinit(MotorHandle_t *motor)
{
    MotorErr_t status;

    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((motor->ops == NULL) || (motor->ops->deinit == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    status = motor->ops->deinit(motor);
    if (status == MOTOR_OK) {
        motor->run_monitor.start_time_ms = 0U;
        motor->run_monitor.is_running = 0U;
        motor->run_monitor.timeout_latched = 0U;
        motor->is_initialized = 0U;
    }
    return status;
}

MotorErr_t Motor_SetOutput(MotorHandle_t *motor, int16_t output)
{
    MotorDirection_t direction;
    MotorErr_t status;

    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((motor->ops == NULL) || (motor->ops->setOutput == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }
    if (motor->run_monitor.timeout_latched != 0U) {
        status = motor->ops->getDriveDirection(motor, &direction);
        if (status != MOTOR_OK) {
            return status;
        }
        if (motor_direction_is_running(direction) != 0U) {
            return MOTOR_ERR_TIMEOUT_LATCHED;
        }
    }

    output = Motor_Clamp(output, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    status = motor->ops->setOutput(motor, output);
    if (status != MOTOR_OK) {
        (void)motor_sync_run_monitor(motor);
        return status;
    }
    return motor_sync_run_monitor(motor);
}

MotorErr_t Motor_SetDirOutput(MotorHandle_t *motor,
                              MotorDirection_t dir,
                              int16_t output)
{
    MotorErr_t status;

    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((motor->ops == NULL) || (motor->ops->setDirOutput == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }
    if ((uint32_t)dir >= (uint32_t)MOTOR_DIR_MAX) {
        return MOTOR_ERR_INVALID_PARAM;
    }
    if ((motor_direction_is_running(dir) != 0U) &&
        (motor->run_monitor.timeout_latched != 0U)) {
        return MOTOR_ERR_TIMEOUT_LATCHED;
    }
    if ((motor_direction_is_running(dir) != 0U) && (s_time_source == NULL)) {
        return MOTOR_ERR_NOT_READY;
    }

    output = Motor_Clamp(output, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    status = motor->ops->setDirOutput(motor, dir, output);
    if (status != MOTOR_OK) {
        /* A concrete driver may have fallen back to COAST after the failure. */
        (void)motor_sync_run_monitor(motor);
        return status;
    }
    return motor_sync_run_monitor(motor);
}

MotorErr_t Motor_ResetPosition(MotorHandle_t *motor, int32_t position)
{
    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((motor->ops == NULL) || (motor->ops->resetPosition == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->resetPosition(motor, position);
}

MotorErr_t Motor_SetTimeSource(MotorTimeSource_t time_source)
{
    if (time_source == NULL) {
        return MOTOR_ERR_NULL_PTR;
    }

    s_time_source = time_source;
    return MOTOR_OK;
}

MotorErr_t Motor_ConfigureRunTimeout(MotorHandle_t *motor,
                                     uint32_t timeout_ms,
                                     MotorDirection_t stop_mode)
{
    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((stop_mode != MOTOR_DIR_COAST) && (stop_mode != MOTOR_DIR_BRAKE)) {
        return MOTOR_ERR_INVALID_PARAM;
    }

    motor->run_monitor.timeout_ms = timeout_ms;
    motor->run_monitor.timeout_stop_mode = stop_mode;
    return MOTOR_OK;
}

MotorErr_t Motor_GetOutput(const MotorHandle_t *motor, int16_t *output)
{
    if ((motor == NULL) || (output == NULL)) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((motor->ops == NULL) || (motor->ops->getOutput == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getOutput(motor, output);
}

MotorErr_t Motor_GetDriveDirection(const MotorHandle_t *motor,
                                   MotorDirection_t *dir)
{
    if ((motor == NULL) || (dir == NULL)) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((motor->ops == NULL) || (motor->ops->getDriveDirection == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getDriveDirection(motor, dir);
}

MotorErr_t Motor_GetMeasuredDirection(const MotorHandle_t *motor,
                                      MotorDirection_t *dir)
{
    if ((motor == NULL) || (dir == NULL)) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((motor->ops == NULL) || (motor->ops->getMeasuredDirection == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getMeasuredDirection(motor, dir);
}

MotorErr_t Motor_GetMeasuredPosition(const MotorHandle_t *motor,
                                     int32_t *position)
{
    if ((motor == NULL) || (position == NULL)) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((motor->ops == NULL) || (motor->ops->getMeasuredPosition == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getMeasuredPosition(motor, position);
}

MotorErr_t Motor_GetMeasuredVelocity(const MotorHandle_t *motor,
                                     float *velocity)
{
    if ((motor == NULL) || (velocity == NULL)) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((motor->ops == NULL) || (motor->ops->getMeasuredVelocity == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getMeasuredVelocity(motor, velocity);
}

MotorErr_t Motor_GetMeasuredCurrent(const MotorHandle_t *motor, float *current)
{
    if ((motor == NULL) || (current == NULL)) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if ((motor->ops == NULL) || (motor->ops->getMeasuredCurrent == NULL)) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

    return motor->ops->getMeasuredCurrent(motor, current);
}

MotorErr_t Motor_GetRunningTime(const MotorHandle_t *motor, uint32_t *time_ms)
{
    if ((motor == NULL) || (time_ms == NULL)) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->run_monitor.is_running == 0U) {
        *time_ms = 0U;
        return MOTOR_OK;
    }
    if (s_time_source == NULL) {
        return MOTOR_ERR_NOT_READY;
    }

    /* Unsigned subtraction remains correct when the 32-bit tick wraps. */
    *time_ms = s_time_source() - motor->run_monitor.start_time_ms;
    return MOTOR_OK;
}

MotorErr_t Motor_GetRunTimeout(const MotorHandle_t *motor, uint8_t *timed_out)
{
    if ((motor == NULL) || (timed_out == NULL)) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;

    *timed_out = motor->run_monitor.timeout_latched;
    return MOTOR_OK;
}

MotorErr_t Motor_ClearRunTimeout(MotorHandle_t *motor)
{
    MotorErr_t status;

    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;

    status = motor_sync_run_monitor(motor);
    if (status != MOTOR_OK) {
        return status;
    }
    if (motor->run_monitor.is_running != 0U) {
        return MOTOR_ERR_NOT_READY;
    }

    motor->run_monitor.timeout_latched = 0U;
    return MOTOR_OK;
}

MotorErr_t Motor_Service(MotorHandle_t *motor)
{
    uint32_t running_time_ms;
    MotorErr_t status;

    if (motor == NULL) return MOTOR_ERR_NULL_PTR;
    if (!motor->is_initialized) return MOTOR_ERR_NOT_INITIALIZED;
    if (motor->run_monitor.timeout_latched != 0U) {
        if (motor->run_monitor.is_running != 0U) {
            return Motor_SetDirOutput(motor,
                                      motor->run_monitor.timeout_stop_mode,
                                      0);
        }
        return MOTOR_OK;
    }
    if ((motor->run_monitor.timeout_ms == 0U) ||
        (motor->run_monitor.is_running == 0U)) {
        return MOTOR_OK;
    }

    status = Motor_GetRunningTime(motor, &running_time_ms);
    if (status != MOTOR_OK) {
        return status;
    }
    if (running_time_ms < motor->run_monitor.timeout_ms) {
        return MOTOR_OK;
    }

    /* Latch first so a failed stop cannot be followed by another run command. */
    motor->run_monitor.timeout_latched = 1U;
    return Motor_SetDirOutput(motor,
                              motor->run_monitor.timeout_stop_mode,
                              0);
}
