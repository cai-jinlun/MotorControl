#include "motor_bdc_vnh.h"
#include <string.h>

#ifndef MOTOR_BDC_VNH_INSTANCE_COUNT
#define MOTOR_BDC_VNH_INSTANCE_COUNT 4U
#endif

#if (MOTOR_BDC_VNH_INSTANCE_COUNT == 0U) || (MOTOR_BDC_VNH_INSTANCE_COUNT > 32U)
#error "MOTOR_BDC_VNH_INSTANCE_COUNT must be in the range 1..32"
#endif

typedef struct {
    MotorBDC_VNH_Config_t config;
    int16_t               output;
    MotorDirection_t      direction;
    uint8_t               init_called;
} MotorBDC_VNH_Private_t;

typedef struct {
    MotorHandle_t         base;
    MotorBDC_VNH_Private_t priv;
} MotorBDC_VNH_Instance_t;

static MotorErr_t bdc_vnh_init(MotorHandle_t *motor);
static MotorErr_t bdc_vnh_deinit(MotorHandle_t *motor);
static MotorErr_t bdc_vnh_set_output(MotorHandle_t *motor, int16_t output);
static MotorErr_t bdc_vnh_set_dir_output(MotorHandle_t *motor,
                                          MotorDirection_t direction,
                                          int16_t output);
static MotorErr_t bdc_vnh_reset_position(MotorHandle_t *motor,
                                          int32_t position);
static MotorErr_t bdc_vnh_get_output(const MotorHandle_t *motor,
                                      int16_t *output);
static MotorErr_t bdc_vnh_get_drive_direction(const MotorHandle_t *motor,
                                               MotorDirection_t *direction);
static MotorErr_t bdc_vnh_get_measured_direction(const MotorHandle_t *motor,
                                                  MotorDirection_t *direction);
static MotorErr_t bdc_vnh_get_measured_position(const MotorHandle_t *motor,
                                                 int32_t *position);
static MotorErr_t bdc_vnh_get_measured_velocity(const MotorHandle_t *motor,
                                                 float *velocity);
static MotorErr_t bdc_vnh_get_measured_current(const MotorHandle_t *motor,
                                                float *current);

static const MotorOps_t s_bdc_vnh_ops = {
    .init                  = bdc_vnh_init,
    .deinit                = bdc_vnh_deinit,
    .setOutput             = bdc_vnh_set_output,
    .setDirOutput          = bdc_vnh_set_dir_output,
    .resetPosition         = bdc_vnh_reset_position,
    .getOutput             = bdc_vnh_get_output,
    .getDriveDirection     = bdc_vnh_get_drive_direction,
    .getMeasuredDirection  = bdc_vnh_get_measured_direction,
    .getMeasuredPosition   = bdc_vnh_get_measured_position,
    .getMeasuredVelocity   = bdc_vnh_get_measured_velocity,
    .getMeasuredCurrent    = bdc_vnh_get_measured_current,
};

static MotorBDC_VNH_Instance_t s_instance_pool[MOTOR_BDC_VNH_INSTANCE_COUNT];
static uint32_t s_instance_map;

/* 检查创建电机所需的板级回调和配置范围是否完整。 */
static uint8_t bdc_vnh_config_valid(const MotorBDC_VNH_Config_t *cfg)
{
    if ((cfg == NULL) || (cfg->port == NULL) ||
        (cfg->dead_zone > MOTOR_OUTPUT_MAX)) {
        return 0U;
    }

    if ((cfg->port->init == NULL) || (cfg->port->deinit == NULL) ||
        (cfg->port->set_outputs == NULL)) {
        return 0U;
    }

    return 1U;
}

/* 在静态实例池中定位有效句柄；用于防止重复或非法销毁。 */
static int32_t bdc_vnh_find_instance(const MotorHandle_t *handle)
{
    uint32_t index;

    for (index = 0U; index < MOTOR_BDC_VNH_INSTANCE_COUNT; ++index) {
        if (((s_instance_map & (1UL << index)) != 0U) &&
            (handle == &s_instance_pool[index].base)) {
            return (int32_t)index;
        }
    }
    return -1;
}

/*
 * 根据方向生成 VNH 的 IN_A/IN_B/PWM 真值表并一次提交给板级层。
 * 正反转使用相反的方向输入；异常时尝试回退 Coast。
 */
static MotorErr_t bdc_vnh_apply(MotorHandle_t *motor,
                                 MotorDirection_t direction,
                                 int16_t output)
{
    MotorBDC_VNH_Private_t *priv;
    uint8_t in_a = 0U;
    uint8_t in_b = 0U;
    int16_t applied_output = output;
    MotorDirection_t applied_direction = direction;
    MotorErr_t status;

    if ((motor == NULL) || (motor->priv == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    if ((uint32_t)direction >= (uint32_t)MOTOR_DIR_MAX) {
        return MOTOR_ERR_INVALID_PARAM;
    }

    priv = (MotorBDC_VNH_Private_t *)motor->priv;

    switch (direction) {
    case MOTOR_DIR_FORWARD:
        in_a = 1U;
        if (applied_output < (int16_t)priv->config.dead_zone) {
            applied_output = 0;
        }
        break;

    case MOTOR_DIR_BACKWARD:
        in_b = 1U;
        if (applied_output < (int16_t)priv->config.dead_zone) {
            applied_output = 0;
        }
        break;

    case MOTOR_DIR_COAST:
        applied_output = 0;
        break;

    case MOTOR_DIR_BRAKE:
        in_a = 1U;
        in_b = 1U;
        applied_output = MOTOR_OUTPUT_MAX;
        break;

    default:
        return MOTOR_ERR_INVALID_PARAM;
    }

    /* 零输出或死区输出没有实际驱动，应缓存为 COAST，避免通用层误计时。 */
    if ((applied_output == 0) &&
        ((direction == MOTOR_DIR_FORWARD) ||
         (direction == MOTOR_DIR_BACKWARD))) {
        in_a = 0U;
        in_b = 0U;
        applied_direction = MOTOR_DIR_COAST;
    }

    status = priv->config.port->set_outputs(priv->config.context,
                                             in_a,
                                             in_b,
                                             (uint16_t)applied_output);
    if (status == MOTOR_OK) {
        priv->direction = applied_direction;
        priv->output = applied_output;
        return MOTOR_OK;
    }

    /* 输出提交失败时尽力回到 Coast：A=0、B=0、PWM=0。 */
    if (priv->config.port->set_outputs(priv->config.context,
                                        0U,
                                        0U,
                                        0U) == MOTOR_OK) {
        priv->direction = MOTOR_DIR_COAST;
        priv->output = 0;
    }
    return status;
}

/* 从静态池创建实例、复制配置并自动进入已初始化的 Coast 状态。 */
MotorHandle_t *MotorBDC_VNH_Create(const MotorBDC_VNH_Config_t *cfg,
                                   MotorErr_t *error)
{
    MotorBDC_VNH_Instance_t *instance;
    MotorErr_t status;
    uint32_t index;

    if (error != NULL) {
        *error = MOTOR_OK;
    }
    if (cfg == NULL) {
        if (error != NULL) {
            *error = MOTOR_ERR_NULL_PTR;
        }
        return NULL;
    }
    if (bdc_vnh_config_valid(cfg) == 0U) {
        if (error != NULL) {
            *error = MOTOR_ERR_INVALID_PARAM;
        }
        return NULL;
    }

    for (index = 0U; index < MOTOR_BDC_VNH_INSTANCE_COUNT; ++index) {
        if ((s_instance_map & (1UL << index)) == 0U) {
            break;
        }
    }
    if (index >= MOTOR_BDC_VNH_INSTANCE_COUNT) {
        if (error != NULL) {
            *error = MOTOR_ERR_NO_RESOURCE;
        }
        return NULL;
    }

    s_instance_map |= (1UL << index);
    instance = &s_instance_pool[index];
    memset(instance, 0, sizeof(*instance));

    instance->base.type = MOTOR_TYPE_BDC_VNH;
    instance->base.ops = &s_bdc_vnh_ops;
    instance->base.priv = &instance->priv;
    instance->priv.config = *cfg;
    instance->priv.direction = MOTOR_DIR_COAST;

    status = Motor_Init(&instance->base);
    if (status != MOTOR_OK) {
        s_instance_map &= ~(1UL << index);
        memset(instance, 0, sizeof(*instance));
        if (error != NULL) {
            *error = status;
        }
        return NULL;
    }

    return &instance->base;
}

/* 销毁有效实例：只有安全反初始化成功后才释放静态池槽位。 */
MotorErr_t MotorBDC_VNH_Destroy(MotorHandle_t *handle)
{
    int32_t index;
    MotorBDC_VNH_Instance_t *instance;
    MotorErr_t status;

    if (handle == NULL) {
        return MOTOR_ERR_NULL_PTR;
    }
    index = bdc_vnh_find_instance(handle);
    if (index < 0) {
        return MOTOR_ERR_INVALID_PARAM;
    }

    instance = &s_instance_pool[(uint32_t)index];
    if (instance->base.is_initialized != 0U) {
        status = Motor_Deinit(&instance->base);
        if (status != MOTOR_OK) {
            return status;
        }
    }

    s_instance_map &= ~(1UL << (uint32_t)index);
    memset(instance, 0, sizeof(*instance));
    return MOTOR_OK;
}

/* 调用板级 init 并提交初始 Coast；失败时按生命周期约定回滚。 */
static MotorErr_t bdc_vnh_init(MotorHandle_t *motor)
{
    MotorBDC_VNH_Private_t *priv;
    MotorErr_t rollback_status;
    MotorErr_t status;

    if ((motor == NULL) || (motor->priv == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    priv = (MotorBDC_VNH_Private_t *)motor->priv;

    if (priv->init_called != 0U) {
        return MOTOR_ERR_ALREADY_INIT;
    }
    priv->init_called = 1U;

    status = priv->config.port->init(priv->config.context);
    if (status != MOTOR_OK) {
        /* 板级 init 必须自行回滚其中途启动的资源。 */
        priv->init_called = 0U;
        return status;
    }

    status = priv->config.port->set_outputs(priv->config.context,
                                             0U,
                                             0U,
                                             0U);
    if (status != MOTOR_OK) {
        /* init 已成功，因此用一次 deinit 平衡板级资源引用。 */
        rollback_status = priv->config.port->deinit(priv->config.context);
        if (rollback_status == MOTOR_OK) {
            priv->init_called = 0U;
        }
        return status;
    }

    priv->direction = MOTOR_DIR_COAST;
    priv->output = 0;
    return MOTOR_OK;
}

/* 先请求 Coast，再用可重试的板级 deinit 强制关断并释放资源。 */
static MotorErr_t bdc_vnh_deinit(MotorHandle_t *motor)
{
    MotorBDC_VNH_Private_t *priv;
    MotorErr_t coast_status;
    MotorErr_t deinit_status;

    if ((motor == NULL) || (motor->priv == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    priv = (MotorBDC_VNH_Private_t *)motor->priv;

    coast_status = priv->config.port->set_outputs(priv->config.context,
                                                   0U,
                                                   0U,
                                                   0U);
    deinit_status = priv->config.port->deinit(priv->config.context);

    if (deinit_status != MOTOR_OK) {
        if (coast_status == MOTOR_OK) {
            priv->direction = MOTOR_DIR_COAST;
            priv->output = 0;
        }
        return deinit_status;
    }

    /* 板级 deinit 已确认强制关断，因此即使前置 Coast 失败也视为成功。 */
    priv->direction = MOTOR_DIR_COAST;
    priv->output = 0;
    priv->init_called = 0U;
    return MOTOR_OK;
}

/* 保持当前方向，仅更新当前方向对应的 PWM 输出。 */
static MotorErr_t bdc_vnh_set_output(MotorHandle_t *motor, int16_t output)
{
    MotorBDC_VNH_Private_t *priv;

    if ((motor == NULL) || (motor->priv == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    priv = (MotorBDC_VNH_Private_t *)motor->priv;
    return bdc_vnh_apply(motor, priv->direction, output);
}

/* 同时设置方向和输出幅值，由 bdc_vnh_apply 生成完整真值表。 */
static MotorErr_t bdc_vnh_set_dir_output(MotorHandle_t *motor,
                                          MotorDirection_t direction,
                                          int16_t output)
{
    return bdc_vnh_apply(motor, direction, output);
}

/* 将位置复位请求转发给可选的板级霍尔/编码器实现。 */
static MotorErr_t bdc_vnh_reset_position(MotorHandle_t *motor, int32_t position)
{
    MotorBDC_VNH_Private_t *priv = (MotorBDC_VNH_Private_t *)motor->priv;

    if (priv->config.port->reset_position == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }
    return priv->config.port->reset_position(priv->config.context, position);
}

/* 读取驱动层缓存的实际归一化输出值。 */
static MotorErr_t bdc_vnh_get_output(const MotorHandle_t *motor,
                                      int16_t *output)
{
    const MotorBDC_VNH_Private_t *priv =
        (const MotorBDC_VNH_Private_t *)motor->priv;
    *output = priv->output;
    return MOTOR_OK;
}

/* 读取最近一次成功提交的驱动方向。 */
static MotorErr_t bdc_vnh_get_drive_direction(const MotorHandle_t *motor,
                                               MotorDirection_t *direction)
{
    const MotorBDC_VNH_Private_t *priv =
        (const MotorBDC_VNH_Private_t *)motor->priv;
    *direction = priv->direction;
    return MOTOR_OK;
}

/* 根据可选的带符号实测速度换算方向；零速统一视为 Coast。 */
static MotorErr_t bdc_vnh_get_measured_direction(const MotorHandle_t *motor,
                                                  MotorDirection_t *direction)
{
    const MotorBDC_VNH_Private_t *priv =
        (const MotorBDC_VNH_Private_t *)motor->priv;
    float velocity;
    MotorErr_t status;

    if (priv->config.port->get_velocity == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }

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

/* 读取可选板级反馈中的原始位置计数。 */
static MotorErr_t bdc_vnh_get_measured_position(const MotorHandle_t *motor,
                                                 int32_t *position)
{
    const MotorBDC_VNH_Private_t *priv =
        (const MotorBDC_VNH_Private_t *)motor->priv;

    if (priv->config.port->get_position == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }
    return priv->config.port->get_position(priv->config.context, position);
}

/* 读取可选板级反馈中的带符号速度。 */
static MotorErr_t bdc_vnh_get_measured_velocity(const MotorHandle_t *motor,
                                                 float *velocity)
{
    const MotorBDC_VNH_Private_t *priv =
        (const MotorBDC_VNH_Private_t *)motor->priv;

    if (priv->config.port->get_velocity == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }
    return priv->config.port->get_velocity(priv->config.context, velocity);
}

/* 读取可选板级反馈中的已标定电流（单位：A）。 */
static MotorErr_t bdc_vnh_get_measured_current(const MotorHandle_t *motor,
                                                float *current)
{
    const MotorBDC_VNH_Private_t *priv =
        (const MotorBDC_VNH_Private_t *)motor->priv;

    if (priv->config.port->get_current == NULL) {
        return MOTOR_ERR_NOT_SUPPORTED;
    }
    return priv->config.port->get_current(priv->config.context, current);
}
