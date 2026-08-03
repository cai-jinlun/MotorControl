#include "motor_bdc_vnh.h"
#include "motor_runtime.h"
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
    MotorRuntime_t        runtime;     /* 运行时间统计（以实际有效输出为判据） */
    uint8_t               init_called;
    uint8_t               deinit_called;
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
static MotorErr_t bdc_vnh_set_running_time(MotorHandle_t *motor,
                                            uint32_t time_ms);
static MotorErr_t bdc_vnh_get_output(const MotorHandle_t *motor,
                                      int16_t *output);
static MotorErr_t bdc_vnh_get_drive_direction(const MotorHandle_t *motor,
                                               MotorDirection_t *direction);
static MotorErr_t bdc_vnh_get_running_time(const MotorHandle_t *motor,
                                            uint32_t *time_ms);

/*
 * VNH 类电机无位置/速度/电流反馈硬件，相关通用 API 条目保持 NULL，
 * 上层调用时将得到 MOTOR_ERR_NOT_SUPPORTED。
 */
static const MotorOps_t s_bdc_vnh_ops = {
    .init                  = bdc_vnh_init,
    .deinit                = bdc_vnh_deinit,
    .setOutput             = bdc_vnh_set_output,
    .setDirOutput          = bdc_vnh_set_dir_output,
    .setRunningTime        = bdc_vnh_set_running_time,
    .getOutput             = bdc_vnh_get_output,
    .getDriveDirection     = bdc_vnh_get_drive_direction,
    .getRunningTime        = bdc_vnh_get_running_time,
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
        (cfg->port->set_outputs == NULL) ||
        (cfg->port->get_time_ms == NULL)) {
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
 * 读取板级毫秒时钟并推进运行时间统计。
 * running 为死区处理后的实际运行状态；时钟读取失败时跳过本次更新，
 * 不影响输出提交结果。
 */
static void bdc_vnh_update_runtime(MotorBDC_VNH_Private_t *priv,
                                    uint8_t running)
{
    uint32_t now_ms;

    if (priv->config.port->get_time_ms(priv->config.context,
                                       &now_ms) == MOTOR_OK) {
        MotorRuntime_Update(&priv->runtime, running, now_ms);
    }
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
    MotorErr_t status;

    if ((motor == NULL) || (motor->priv == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    if ((uint32_t)direction >= (uint32_t)MOTOR_DIR_MAX) {
        return MOTOR_ERR_INVALID_PARAM;
    }

    priv = (MotorBDC_VNH_Private_t *)motor->priv;
    if (applied_output < MOTOR_OUTPUT_MIN) {
        applied_output = MOTOR_OUTPUT_MIN;
    } else if (applied_output > MOTOR_OUTPUT_MAX) {
        applied_output = MOTOR_OUTPUT_MAX;
    }

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

    status = priv->config.port->set_outputs(priv->config.context,
                                             in_a,
                                             in_b,
                                             (uint16_t)applied_output);
    if (status == MOTOR_OK) {
        uint8_t running = 0U;

        if (((direction == MOTOR_DIR_FORWARD) ||
             (direction == MOTOR_DIR_BACKWARD)) &&
            (applied_output > 0)) {
            running = 1U;
        }
        bdc_vnh_update_runtime(priv, running);

        priv->direction = direction;
        priv->output = applied_output;
        return MOTOR_OK;
    }

    /* 输出提交失败时尽力回到 Coast：A=0、B=0、PWM=0。 */
    if (priv->config.port->set_outputs(priv->config.context,
                                        0U,
                                        0U,
                                        0U) == MOTOR_OK) {
        bdc_vnh_update_runtime(priv, 0U);
        priv->direction = MOTOR_DIR_COAST;
        priv->output = 0;
    }
    return status;
}

/* 向通用电机框架注册 BDC_VNH 的操作表。 */
MotorErr_t MotorBDC_VNH_ModuleInit(void)
{
    return Motor_RegisterOps(MOTOR_TYPE_BDC_VNH, &s_bdc_vnh_ops);
}

/* 从静态池创建实例、复制配置并自动进入已初始化的 Coast 状态。 */
MotorHandle_t *MotorBDC_VNH_Create(const MotorBDC_VNH_Config_t *cfg)
{
    MotorBDC_VNH_Instance_t *instance;
    uint32_t index;

    if (bdc_vnh_config_valid(cfg) == 0U) {
        return NULL;
    }

    for (index = 0U; index < MOTOR_BDC_VNH_INSTANCE_COUNT; ++index) {
        if ((s_instance_map & (1UL << index)) == 0U) {
            break;
        }
    }
    if (index >= MOTOR_BDC_VNH_INSTANCE_COUNT) {
        return NULL;
    }

    s_instance_map |= (1UL << index);
    instance = &s_instance_pool[index];
    memset(instance, 0, sizeof(*instance));

    instance->base.id = (uint8_t)index;
    instance->base.type = MOTOR_TYPE_BDC_VNH;
    instance->base.ops = &s_bdc_vnh_ops;
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

/* 销毁有效实例：先执行一次反初始化，再释放静态池槽位。 */
void MotorBDC_VNH_Destroy(MotorHandle_t *handle)
{
    int32_t index = bdc_vnh_find_instance(handle);
    MotorBDC_VNH_Instance_t *instance;

    if (index < 0) {
        return;
    }

    instance = &s_instance_pool[(uint32_t)index];
    if (instance->base.is_initialized != 0U) {
        (void)Motor_Deinit(&instance->base);
    }

    s_instance_map &= ~(1UL << (uint32_t)index);
    memset(instance, 0, sizeof(*instance));
}

/* 调用板级 init 并提交初始 Coast；失败时按生命周期约定回滚。 */
static MotorErr_t bdc_vnh_init(MotorHandle_t *motor)
{
    MotorBDC_VNH_Private_t *priv;
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
        return status;
    }

    status = priv->config.port->set_outputs(priv->config.context,
                                             0U,
                                             0U,
                                             0U);
    if (status != MOTOR_OK) {
        /* init 已成功，因此用一次 deinit 平衡板级资源引用。 */
        priv->deinit_called = 1U;
        (void)priv->config.port->deinit(priv->config.context);
        return status;
    }

    priv->direction = MOTOR_DIR_COAST;
    priv->output = 0;
    return MOTOR_OK;
}

/* 先请求 Coast，再仅一次调用板级 deinit 释放资源引用。 */
static MotorErr_t bdc_vnh_deinit(MotorHandle_t *motor)
{
    MotorBDC_VNH_Private_t *priv;
    MotorErr_t coast_status;
    MotorErr_t deinit_status;

    if ((motor == NULL) || (motor->priv == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    priv = (MotorBDC_VNH_Private_t *)motor->priv;

    if (priv->deinit_called != 0U) {
        return MOTOR_OK;
    }

    coast_status = priv->config.port->set_outputs(priv->config.context,
                                                   0U,
                                                   0U,
                                                   0U);
    bdc_vnh_update_runtime(priv, 0U);
    priv->deinit_called = 1U;
    deinit_status = priv->config.port->deinit(priv->config.context);
    priv->direction = MOTOR_DIR_COAST;
    priv->output = 0;

    return (coast_status != MOTOR_OK) ? coast_status : deinit_status;
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

/*
 * 重置/设置累计运行时间。运行中调用时，当前运行段从设定值起继续累加。
 */
static MotorErr_t bdc_vnh_set_running_time(MotorHandle_t *motor,
                                            uint32_t time_ms)
{
    MotorBDC_VNH_Private_t *priv =
        (MotorBDC_VNH_Private_t *)motor->priv;
    uint32_t now_ms;
    MotorErr_t status;

    status = priv->config.port->get_time_ms(priv->config.context, &now_ms);
    if (status != MOTOR_OK) {
        return status;
    }

    MotorRuntime_Set(&priv->runtime, time_ms, now_ms);
    return MOTOR_OK;
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

/* 读取累计运行时间（ms），运行中读取会实时结算当前运行段。 */
static MotorErr_t bdc_vnh_get_running_time(const MotorHandle_t *motor,
                                            uint32_t *time_ms)
{
    const MotorBDC_VNH_Private_t *priv =
        (const MotorBDC_VNH_Private_t *)motor->priv;
    uint32_t now_ms;
    MotorErr_t status;

    status = priv->config.port->get_time_ms(priv->config.context, &now_ms);
    if (status != MOTOR_OK) {
        return status;
    }

    *time_ms = MotorRuntime_Get(&priv->runtime, now_ms);
    return MOTOR_OK;
}
