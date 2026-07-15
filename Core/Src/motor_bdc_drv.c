#include "motor_bdc_drv.h"
#include <string.h>

#ifndef MOTOR_BDC_DRV_INSTANCE_COUNT
#define MOTOR_BDC_DRV_INSTANCE_COUNT 4U
#endif

#if (MOTOR_BDC_DRV_INSTANCE_COUNT == 0U) || (MOTOR_BDC_DRV_INSTANCE_COUNT > 32U)
#error "MOTOR_BDC_DRV_INSTANCE_COUNT must be in the range 1..32"
#endif

typedef struct {
    MotorBDC_DRV_Config_t config;
    int16_t               output;
    MotorDirection_t      direction;
    uint8_t               init_called;
    uint8_t               deinit_called;
} MotorBDC_DRV_Private_t;

typedef struct {
    MotorHandle_t         base;
    MotorBDC_DRV_Private_t priv;
} MotorBDC_DRV_Instance_t;

static MotorErr_t bdc_drv_init(MotorHandle_t *motor);
static MotorErr_t bdc_drv_deinit(MotorHandle_t *motor);
static MotorErr_t bdc_drv_set_output(MotorHandle_t *motor, int16_t output);
static MotorErr_t bdc_drv_set_dir_output(MotorHandle_t *motor,
                                          MotorDirection_t direction,
                                          int16_t output);
static MotorErr_t bdc_drv_reset_position(MotorHandle_t *motor,
                                          int32_t position);
static MotorErr_t bdc_drv_get_output(const MotorHandle_t *motor,
                                      int16_t *output);
static MotorErr_t bdc_drv_get_drive_direction(const MotorHandle_t *motor,
                                               MotorDirection_t *direction);
static MotorErr_t bdc_drv_get_measured_direction(const MotorHandle_t *motor,
                                                  MotorDirection_t *direction);
static MotorErr_t bdc_drv_get_measured_position(const MotorHandle_t *motor,
                                                 int32_t *position);
static MotorErr_t bdc_drv_get_measured_velocity(const MotorHandle_t *motor,
                                                 float *velocity);
static MotorErr_t bdc_drv_get_measured_current(const MotorHandle_t *motor,
                                                float *current);

static const MotorOps_t s_bdc_drv_ops = {
    .init                  = bdc_drv_init,
    .deinit                = bdc_drv_deinit,
    .setOutput             = bdc_drv_set_output,
    .setDirOutput          = bdc_drv_set_dir_output,
    .resetPosition         = bdc_drv_reset_position,
    .getOutput             = bdc_drv_get_output,
    .getDriveDirection     = bdc_drv_get_drive_direction,
    .getMeasuredDirection  = bdc_drv_get_measured_direction,
    .getMeasuredPosition   = bdc_drv_get_measured_position,
    .getMeasuredVelocity   = bdc_drv_get_measured_velocity,
    .getMeasuredCurrent    = bdc_drv_get_measured_current,
};

static MotorBDC_DRV_Instance_t s_instance_pool[MOTOR_BDC_DRV_INSTANCE_COUNT];
static uint32_t s_instance_map;

static uint8_t bdc_drv_config_valid(const MotorBDC_DRV_Config_t *cfg)
{
    const MotorBDC_DRV_PortOps_t *port;

    if ((cfg == NULL) || (cfg->port == NULL) ||
        (cfg->dead_zone > MOTOR_OUTPUT_MAX)) {
        return 0U;
    }

    port = cfg->port;
    if ((port->init == NULL) || (port->deinit == NULL) ||
        (port->set_inputs == NULL) || (port->get_position == NULL) ||
        (port->reset_position == NULL) || (port->get_velocity == NULL) ||
        (port->get_current == NULL)) {
        return 0U;
    }

    return 1U;
}

static int32_t bdc_drv_find_instance(const MotorHandle_t *handle)
{
    uint32_t index;

    for (index = 0U; index < MOTOR_BDC_DRV_INSTANCE_COUNT; ++index) {
        if (((s_instance_map & (1UL << index)) != 0U) &&
            (handle == &s_instance_pool[index].base)) {
            return (int32_t)index;
        }
    }
    return -1;
}

static MotorErr_t bdc_drv_apply(MotorHandle_t *motor,
                                 MotorDirection_t direction,
                                 int16_t output)
{
    MotorBDC_DRV_Private_t *priv;
    uint16_t in1 = 0U;
    uint16_t in2 = 0U;
    int16_t applied_output = output;
    MotorErr_t status;

    if ((motor == NULL) || (motor->priv == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    if ((uint32_t)direction >= (uint32_t)MOTOR_DIR_MAX) {
        return MOTOR_ERR_INVALID_PARAM;
    }

    priv = (MotorBDC_DRV_Private_t *)motor->priv;
    if (applied_output < MOTOR_OUTPUT_MIN) {
        applied_output = MOTOR_OUTPUT_MIN;
    } else if (applied_output > MOTOR_OUTPUT_MAX) {
        applied_output = MOTOR_OUTPUT_MAX;
    }

    switch (direction) {
    case MOTOR_DIR_FORWARD:
        if (applied_output < (int16_t)priv->config.dead_zone) {
            applied_output = 0;
        }
        in1 = (uint16_t)applied_output;
        break;

    case MOTOR_DIR_BACKWARD:
        if (applied_output < (int16_t)priv->config.dead_zone) {
            applied_output = 0;
        }
        in2 = (uint16_t)applied_output;
        break;

    case MOTOR_DIR_COAST:
        applied_output = 0;
        break;

    case MOTOR_DIR_BRAKE:
        in1 = MOTOR_OUTPUT_MAX;
        in2 = MOTOR_OUTPUT_MAX;
        applied_output = MOTOR_OUTPUT_MAX;
        break;

    default:
        return MOTOR_ERR_INVALID_PARAM;
    }

    status = priv->config.port->set_inputs(priv->config.context, in1, in2);
    if (status == MOTOR_OK) {
        priv->direction = direction;
        priv->output = applied_output;
        return MOTOR_OK;
    }

    /* 输出提交失败时尽力回到安全的 Coast 状态。 */
    if (priv->config.port->set_inputs(priv->config.context, 0U, 0U) == MOTOR_OK) {
        priv->direction = MOTOR_DIR_COAST;
        priv->output = 0;
    }
    return status;
}

MotorErr_t MotorBDC_DRV_ModuleInit(void)
{
    return Motor_RegisterOps(MOTOR_TYPE_BDC_DRV, &s_bdc_drv_ops);
}

MotorHandle_t *MotorBDC_DRV_Create(const MotorBDC_DRV_Config_t *cfg)
{
    MotorBDC_DRV_Instance_t *instance;
    uint32_t index;

    if (bdc_drv_config_valid(cfg) == 0U) {
        return NULL;
    }

    for (index = 0U; index < MOTOR_BDC_DRV_INSTANCE_COUNT; ++index) {
        if ((s_instance_map & (1UL << index)) == 0U) {
            break;
        }
    }
    if (index >= MOTOR_BDC_DRV_INSTANCE_COUNT) {
        return NULL;
    }

    s_instance_map |= (1UL << index);
    instance = &s_instance_pool[index];
    memset(instance, 0, sizeof(*instance));

    instance->base.id = (uint8_t)index;
    instance->base.type = MOTOR_TYPE_BDC_DRV;
    instance->base.ops = &s_bdc_drv_ops;
    instance->base.priv = &instance->priv;
    instance->priv.config = *cfg;
    instance->priv.direction = MOTOR_DIR_COAST;

    if (Motor_Init(&instance->base) != MOTOR_OK) {
        s_instance_map &= ~(1UL << index);
        memset(instance, 0, sizeof(*instance));
        return NULL;
    }

    return &instance->base;
}

void MotorBDC_DRV_Destroy(MotorHandle_t *handle)
{
    int32_t index = bdc_drv_find_instance(handle);
    MotorBDC_DRV_Instance_t *instance;

    if (index < 0) {
        return;
    }

    instance = &s_instance_pool[(uint32_t)index];
    if (instance->base.is_initialized != 0U) {
        /* Motor_Deinit 内部只会调用一次端口 deinit。 */
        (void)Motor_Deinit(&instance->base);
    }

    s_instance_map &= ~(1UL << (uint32_t)index);
    memset(instance, 0, sizeof(*instance));
}

static MotorErr_t bdc_drv_init(MotorHandle_t *motor)
{
    MotorBDC_DRV_Private_t *priv;
    MotorErr_t status;

    if ((motor == NULL) || (motor->priv == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    priv = (MotorBDC_DRV_Private_t *)motor->priv;

    /* 一个实例的完整生命周期中，板级 init 最多调用一次。 */
    if (priv->init_called != 0U) {
        return MOTOR_ERR_ALREADY_INIT;
    }
    priv->init_called = 1U;

    status = priv->config.port->init(priv->config.context);
    if (status != MOTOR_OK) {
        /* init 失败的内部回滚完全由板级 init 负责。 */
        return status;
    }

    status = priv->config.port->set_inputs(priv->config.context, 0U, 0U);
    if (status != MOTOR_OK) {
        /* init 已成功，必须用恰好一次 deinit 平衡其资源引用。 */
        priv->deinit_called = 1U;
        (void)priv->config.port->deinit(priv->config.context);
        return status;
    }

    priv->direction = MOTOR_DIR_COAST;
    priv->output = 0;
    return MOTOR_OK;
}

static MotorErr_t bdc_drv_deinit(MotorHandle_t *motor)
{
    MotorBDC_DRV_Private_t *priv;
    MotorErr_t coast_status;
    MotorErr_t deinit_status;

    if ((motor == NULL) || (motor->priv == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    priv = (MotorBDC_DRV_Private_t *)motor->priv;

    /* 即使上一次 deinit 报错，也禁止再次释放同一份板级引用。 */
    if (priv->deinit_called != 0U) {
        return MOTOR_OK;
    }

    coast_status = priv->config.port->set_inputs(priv->config.context, 0U, 0U);
    priv->deinit_called = 1U;
    deinit_status = priv->config.port->deinit(priv->config.context);
    priv->direction = MOTOR_DIR_COAST;
    priv->output = 0;

    return (coast_status != MOTOR_OK) ? coast_status : deinit_status;
}

static MotorErr_t bdc_drv_set_output(MotorHandle_t *motor, int16_t output)
{
    MotorBDC_DRV_Private_t *priv;

    if ((motor == NULL) || (motor->priv == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    priv = (MotorBDC_DRV_Private_t *)motor->priv;
    return bdc_drv_apply(motor, priv->direction, output);
}

static MotorErr_t bdc_drv_set_dir_output(MotorHandle_t *motor,
                                          MotorDirection_t direction,
                                          int16_t output)
{
    return bdc_drv_apply(motor, direction, output);
}

static MotorErr_t bdc_drv_reset_position(MotorHandle_t *motor, int32_t position)
{
    MotorBDC_DRV_Private_t *priv = (MotorBDC_DRV_Private_t *)motor->priv;
    return priv->config.port->reset_position(priv->config.context, position);
}

static MotorErr_t bdc_drv_get_output(const MotorHandle_t *motor, int16_t *output)
{
    const MotorBDC_DRV_Private_t *priv =
        (const MotorBDC_DRV_Private_t *)motor->priv;
    *output = priv->output;
    return MOTOR_OK;
}

static MotorErr_t bdc_drv_get_drive_direction(const MotorHandle_t *motor,
                                               MotorDirection_t *direction)
{
    const MotorBDC_DRV_Private_t *priv =
        (const MotorBDC_DRV_Private_t *)motor->priv;
    *direction = priv->direction;
    return MOTOR_OK;
}

static MotorErr_t bdc_drv_get_measured_direction(const MotorHandle_t *motor,
                                                  MotorDirection_t *direction)
{
    const MotorBDC_DRV_Private_t *priv =
        (const MotorBDC_DRV_Private_t *)motor->priv;
    float velocity;
    MotorErr_t status;

    status = priv->config.port->get_velocity(priv->config.context, &velocity);
    if (status != MOTOR_OK) {
        return status;
    }

    if (velocity > 0.0f) {
        *direction = MOTOR_DIR_FORWARD;
    } else if (velocity < 0.0f) {
        *direction = MOTOR_DIR_BACKWARD;
    } else {
        *direction = MOTOR_DIR_COAST;
    }
    return MOTOR_OK;
}

static MotorErr_t bdc_drv_get_measured_position(const MotorHandle_t *motor,
                                                 int32_t *position)
{
    const MotorBDC_DRV_Private_t *priv =
        (const MotorBDC_DRV_Private_t *)motor->priv;
    return priv->config.port->get_position(priv->config.context, position);
}

static MotorErr_t bdc_drv_get_measured_velocity(const MotorHandle_t *motor,
                                                 float *velocity)
{
    const MotorBDC_DRV_Private_t *priv =
        (const MotorBDC_DRV_Private_t *)motor->priv;
    return priv->config.port->get_velocity(priv->config.context, velocity);
}

static MotorErr_t bdc_drv_get_measured_current(const MotorHandle_t *motor,
                                                float *current)
{
    const MotorBDC_DRV_Private_t *priv =
        (const MotorBDC_DRV_Private_t *)motor->priv;
    return priv->config.port->get_current(priv->config.context, current);
}
