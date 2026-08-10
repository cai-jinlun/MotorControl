#include "motor_driver.h"

#include <assert.h>
#include <stdint.h>

typedef struct {
    MotorDirection_t direction;
    int16_t output;
    uint8_t fail_next_stop;
} FakeMotor_t;

static uint32_t s_tick_ms;

static uint32_t fake_time_ms(void)
{
    return s_tick_ms;
}

static MotorErr_t fake_init(MotorHandle_t *motor)
{
    FakeMotor_t *fake = (FakeMotor_t *)motor->priv;
    fake->direction = MOTOR_DIR_COAST;
    fake->output = 0;
    return MOTOR_OK;
}

static MotorErr_t fake_deinit(MotorHandle_t *motor)
{
    return fake_init(motor);
}

static MotorErr_t fake_set_output(MotorHandle_t *motor, int16_t output)
{
    FakeMotor_t *fake = (FakeMotor_t *)motor->priv;

    fake->output = output;
    if ((output == 0) &&
        ((fake->direction == MOTOR_DIR_FORWARD) ||
         (fake->direction == MOTOR_DIR_BACKWARD))) {
        fake->direction = MOTOR_DIR_COAST;
    }
    return MOTOR_OK;
}

static MotorErr_t fake_set_dir_output(MotorHandle_t *motor,
                                      MotorDirection_t direction,
                                      int16_t output)
{
    FakeMotor_t *fake = (FakeMotor_t *)motor->priv;

    if (((direction == MOTOR_DIR_COAST) ||
         (direction == MOTOR_DIR_BRAKE)) &&
        (fake->fail_next_stop != 0U)) {
        fake->fail_next_stop = 0U;
        return MOTOR_ERR_HW_FAILURE;
    }

    fake->direction = direction;
    fake->output = output;
    return MOTOR_OK;
}

static MotorErr_t fake_get_direction(const MotorHandle_t *motor,
                                     MotorDirection_t *direction)
{
    const FakeMotor_t *fake = (const FakeMotor_t *)motor->priv;
    *direction = fake->direction;
    return MOTOR_OK;
}

static const MotorOps_t s_fake_ops = {
    .init = fake_init,
    .deinit = fake_deinit,
    .setOutput = fake_set_output,
    .setDirOutput = fake_set_dir_output,
    .getDriveDirection = fake_get_direction,
};

int main(void)
{
    FakeMotor_t fake = {0};
    MotorHandle_t motor = {
        .type = MOTOR_TYPE_BDC_VNH,
        .ops = &s_fake_ops,
        .priv = &fake,
    };
    MotorDirection_t direction;
    uint32_t elapsed_ms;
    uint8_t timed_out;

    assert(Motor_SetTimeSource(fake_time_ms) == MOTOR_OK);
    assert(Motor_Init(&motor) == MOTOR_OK);
    assert(Motor_ConfigureRunTimeout(&motor, 100U, MOTOR_DIR_BRAKE) == MOTOR_OK);

    s_tick_ms = 100U;
    assert(Motor_SetDirOutput(&motor, MOTOR_DIR_FORWARD, 500) == MOTOR_OK);
    s_tick_ms = 150U;
    assert(Motor_GetRunningTime(&motor, &elapsed_ms) == MOTOR_OK);
    assert(elapsed_ms == 50U);

    assert(Motor_SetDirOutput(&motor, MOTOR_DIR_BACKWARD, 500) == MOTOR_OK);
    s_tick_ms = 160U;
    assert(Motor_GetRunningTime(&motor, &elapsed_ms) == MOTOR_OK);
    assert(elapsed_ms == 60U);
    assert(Motor_ClearRunTimeout(&motor) == MOTOR_ERR_NOT_READY);

    assert(Motor_SetOutput(&motor, 0) == MOTOR_OK);
    assert(Motor_GetRunningTime(&motor, &elapsed_ms) == MOTOR_OK);
    assert(elapsed_ms == 0U);

    s_tick_ms = UINT32_MAX - 5U;
    assert(Motor_SetDirOutput(&motor, MOTOR_DIR_FORWARD, 500) == MOTOR_OK);
    s_tick_ms = 5U;
    assert(Motor_GetRunningTime(&motor, &elapsed_ms) == MOTOR_OK);
    assert(elapsed_ms == 11U);
    assert(Motor_SetDirOutput(&motor, MOTOR_DIR_COAST, 0) == MOTOR_OK);

    assert(Motor_ConfigureRunTimeout(&motor, 10U, MOTOR_DIR_BRAKE) == MOTOR_OK);
    s_tick_ms = 200U;
    assert(Motor_SetDirOutput(&motor, MOTOR_DIR_FORWARD, 500) == MOTOR_OK);
    s_tick_ms = 209U;
    assert(Motor_Service(&motor) == MOTOR_OK);
    assert(fake.direction == MOTOR_DIR_FORWARD);
    s_tick_ms = 210U;
    assert(Motor_Service(&motor) == MOTOR_OK);
    assert(fake.direction == MOTOR_DIR_BRAKE);
    assert(Motor_GetRunTimeout(&motor, &timed_out) == MOTOR_OK);
    assert(timed_out == 1U);
    assert(Motor_SetOutput(&motor, 123) == MOTOR_OK);
    assert(fake.direction == MOTOR_DIR_BRAKE);
    assert(Motor_SetDirOutput(&motor, MOTOR_DIR_FORWARD, 500) ==
           MOTOR_ERR_TIMEOUT_LATCHED);
    assert(Motor_ClearRunTimeout(&motor) == MOTOR_OK);

    assert(Motor_ConfigureRunTimeout(&motor, 1U, MOTOR_DIR_COAST) == MOTOR_OK);
    s_tick_ms = 300U;
    assert(Motor_SetDirOutput(&motor, MOTOR_DIR_FORWARD, 500) == MOTOR_OK);
    fake.fail_next_stop = 1U;
    s_tick_ms = 301U;
    assert(Motor_Service(&motor) == MOTOR_ERR_HW_FAILURE);
    assert(fake.direction == MOTOR_DIR_FORWARD);
    assert(Motor_Service(&motor) == MOTOR_OK);
    assert(fake.direction == MOTOR_DIR_COAST);
    assert(Motor_ClearRunTimeout(&motor) == MOTOR_OK);

    assert(Motor_ConfigureRunTimeout(&motor, 100U, MOTOR_DIR_COAST) == MOTOR_OK);
    s_tick_ms = 400U;
    assert(Motor_SetDirOutput(&motor, MOTOR_DIR_FORWARD, 500) == MOTOR_OK);
    s_tick_ms = 405U;
    assert(Motor_ConfigureRunTimeout(&motor, 4U, MOTOR_DIR_COAST) == MOTOR_OK);
    assert(Motor_Service(&motor) == MOTOR_OK);
    assert(fake.direction == MOTOR_DIR_COAST);
    assert(Motor_ClearRunTimeout(&motor) == MOTOR_OK);

    assert(Motor_ConfigureRunTimeout(&motor, 0U, MOTOR_DIR_COAST) == MOTOR_OK);
    s_tick_ms = 500U;
    assert(Motor_SetDirOutput(&motor, MOTOR_DIR_FORWARD, 500) == MOTOR_OK);
    s_tick_ms = 10000U;
    assert(Motor_Service(&motor) == MOTOR_OK);
    assert(Motor_GetDriveDirection(&motor, &direction) == MOTOR_OK);
    assert(direction == MOTOR_DIR_FORWARD);

    assert(Motor_SetDirOutput(&motor, MOTOR_DIR_COAST, 0) == MOTOR_OK);
    assert(Motor_Deinit(&motor) == MOTOR_OK);
    return 0;
}
