#include "motor_bdc_vnh.h"
#include <string.h>
#include <stdlib.h>   /* for abs() */

/* =====================================================================
 * 私有结构体（对外完全隐藏）
 * ===================================================================== */
struct BdcVnhMotorPriv {
    const MotorBDC_VNH_Config_t *cfg;        /* 配置引用（只读） */
    int16_t                 speed;      /* 当前速度缓存 */
    MotorDirection_t        dir;        /* 当前方向缓存 */
    uint8_t                 pwm_active; /* PWM是否已启动 */
};

/* =====================================================================
 * 静态内存池（避免堆分配，可预测内存占用）,本项目只会用到1~2个实例，预留4个足够了
 * ===================================================================== */
#define MOTOR_BDC_VNH_INSTANCE_COUNT 4

typedef struct {
    MotorHandle_t      base;   /* 必须放在第一个成员！ */
    struct BdcVnhMotorPriv priv;
} BdcVnhMotorInstance_t;

static BdcVnhMotorInstance_t s_bdc_vnh_pool[MOTOR_BDC_VNH_INSTANCE_COUNT];
static uint32_t          s_bdc_vnh_pool_map = 0;   /* 位图：0=空闲，1=占用 */

/* =====================================================================
 * 操作表前置声明
 * ===================================================================== */
static MotorErr_t bdc_vnh_init(MotorHandle_t *m);
static MotorErr_t bdc_vnh_deinit(MotorHandle_t *m);
static MotorErr_t bdc_vnh_setSpeed(MotorHandle_t *m, int16_t speed);
static MotorErr_t bdc_vnh_setDirSpeed(MotorHandle_t *m, MotorDirection_t dir, int16_t speed);
static const MotorOps_t s_bdc_vnh_ops = {
    .init         = bdc_vnh_init,
    .deinit       = bdc_vnh_deinit,
    .setSpeed     = bdc_vnh_setSpeed,
    .setDirSpeed = bdc_vnh_setDirSpeed,
    // .getStatus    = NULL,
};

/* =====================================================================
 * 模块初始化
 * ===================================================================== */
void MotorBDC_VNH_ModuleInit(void)
{
    Motor_RegisterOps(MOTOR_TYPE_BDC_VNH, &s_bdc_vnh_ops);
}

/* =====================================================================
 * 工厂函数
 * ===================================================================== */
MotorHandle_t* MotorBDC_VNH_Create(const MotorBDC_VNH_Config_t *cfg)
{
    if (cfg == NULL) return NULL;
    if (cfg->htim == NULL) return NULL;
    if (cfg->pwm_full_scale == 0) return NULL;

    /* 1. 从静态池查找空闲槽 */
    int idx = -1;
    for (int i = 0; i < MOTOR_BDC_VNH_INSTANCE_COUNT; i++) {
        if ((s_bdc_vnh_pool_map & (1UL << i)) == 0) {
            idx = i;
            break;
        }
    }
    if (idx < 0) return NULL;   /* 池耗尽 */

    /* 2. 标记占用并清零 */
    s_bdc_vnh_pool_map |= (1UL << idx);
    BdcVnhMotorInstance_t *inst = &s_bdc_vnh_pool[idx];
    memset(inst, 0, sizeof(*inst));

    /* 3. 初始化基类（MotorHandle_t） */
    inst->base.id             = (uint8_t)idx;
    inst->base.type           = MOTOR_TYPE_BDC_VNH;
    inst->base.is_initialized = 0;
    inst->base.ops            = &s_bdc_vnh_ops;
    inst->base.priv           = &inst->priv;   /* 指向自己的私有部分 */

    /* 4. 初始化私有数据 */
    inst->priv.cfg        = cfg;
    inst->priv.speed      = 0;
    inst->priv.dir        = MOTOR_DIR_COAST;
    inst->priv.pwm_active = 0;

    /* 5. 自动初始化硬件（失败则回滚） */
    if (Motor_Init(&inst->base) != MOTOR_OK) {
        s_bdc_vnh_pool_map &= ~(1UL << idx);
        memset(inst, 0, sizeof(*inst));
        return NULL;
    }

    return &inst->base;   /* 返回基类指针，外部无法直接访问 priv */
}

/* =====================================================================
 * 销毁函数，可能用不到
 * ===================================================================== */
void MotorBDC_VNH_Destroy(MotorHandle_t *handle)
{
    if (handle == NULL) return;

    /* 反初始化硬件 */
    if (handle->ops && handle->ops->deinit) {
        handle->ops->deinit(handle);
    }

    /* 通过基地址计算索引（标准C保证：指向结构体的指针 == 指向其首成员的指针） */
    BdcVnhMotorInstance_t *inst = (BdcVnhMotorInstance_t *)handle;
    int idx = (int)(inst - s_bdc_vnh_pool);

    if (idx >= 0 && idx < MOTOR_BDC_VNH_INSTANCE_COUNT) {
        s_bdc_vnh_pool_map &= ~(1UL << idx);
        memset(inst, 0, sizeof(*inst));
    }
}

/* =====================================================================
 * 操作表具体实现
 * ===================================================================== */

/* 硬件初始化：默认置为悬停状态 */
static MotorErr_t bdc_vnh_init(MotorHandle_t *m)
{
    struct BdcVnhMotorPriv *priv = (struct BdcVnhMotorPriv *)m->priv;
    const MotorBDC_VNH_Config_t *cfg = priv->cfg;

    /* GPIO默认低电平（悬停） */
    HAL_GPIO_WritePin(cfg->gpio_port_a, cfg->gpio_pin_a, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(cfg->gpio_port_b, cfg->gpio_pin_b, GPIO_PIN_RESET);

    /* 启动PWM，初始占空比0 */
    if (HAL_TIM_PWM_Start(cfg->htim, cfg->tim_channel) != HAL_OK) {
        return MOTOR_ERR_HW_FAILURE;
    }
    __HAL_TIM_SET_COMPARE(cfg->htim, cfg->tim_channel, 0);
    priv->pwm_active = 1;

    return MOTOR_OK;
}

/* 硬件反初始化 */
static MotorErr_t bdc_vnh_deinit(MotorHandle_t *m)
{
    struct BdcVnhMotorPriv *priv = (struct BdcVnhMotorPriv *)m->priv;
    const MotorBDC_VNH_Config_t *cfg = priv->cfg;

    /* 先进入悬停，再关PWM */
    HAL_GPIO_WritePin(cfg->gpio_port_a, cfg->gpio_pin_a, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(cfg->gpio_port_b, cfg->gpio_pin_b, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(cfg->htim, cfg->tim_channel, 0);

    HAL_TIM_PWM_Stop(cfg->htim, cfg->tim_channel);
    priv->pwm_active = 0;

    return MOTOR_OK;
}

/* 设置速度：带符号映射 + 死区处理 */
static MotorErr_t bdc_vnh_setSpeed(MotorHandle_t *m, int16_t speed)
{
    struct BdcVnhMotorPriv *priv = (struct BdcVnhMotorPriv *)m->priv;
    const MotorBDC_VNH_Config_t *cfg = priv->cfg;

    /* 死区处理 */
    if (speed < cfg->dead_zone) {
        speed = 0;
    }

    priv->speed = speed;

    /* 线性映射：speed(0~1000) -> PWM(0~pwm_full_scale) */
    uint32_t cmp = ((uint32_t)speed * cfg->pwm_full_scale) / MOTOR_SPEED_MAX;
    if (cmp > cfg->pwm_full_scale) cmp = cfg->pwm_full_scale;  //二次保险，可以删除，API接口处已判断

    __HAL_TIM_SET_COMPARE(cfg->htim, cfg->tim_channel, cmp);
    return MOTOR_OK;
}

/* 设置方向和速度：H桥真值表 有问题 */
static MotorErr_t bdc_vnh_setDirSpeed(MotorHandle_t *m, MotorDirection_t dir, int16_t speed)
{
    struct BdcVnhMotorPriv *priv = (struct BdcVnhMotorPriv *)m->priv;
    const MotorBDC_VNH_Config_t *cfg = priv->cfg;

    priv->dir = dir;

    switch (dir) {
        case MOTOR_DIR_FORWARD:
            HAL_GPIO_WritePin(cfg->gpio_port_a, cfg->gpio_pin_a, GPIO_PIN_SET);
            HAL_GPIO_WritePin(cfg->gpio_port_b, cfg->gpio_pin_b, GPIO_PIN_RESET);
            bdc_vnh_setSpeed(m, speed);
            break;

        case MOTOR_DIR_BACKWARD:
            HAL_GPIO_WritePin(cfg->gpio_port_a, cfg->gpio_pin_a, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(cfg->gpio_port_b, cfg->gpio_pin_b, GPIO_PIN_SET);
            bdc_vnh_setSpeed(m, speed);
            break;

        case MOTOR_DIR_COAST:
            HAL_GPIO_WritePin(cfg->gpio_port_a, cfg->gpio_pin_a, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(cfg->gpio_port_b, cfg->gpio_pin_b, GPIO_PIN_RESET);
            bdc_vnh_setSpeed(m, 0);
            break;

        case MOTOR_DIR_BRAKE:
            HAL_GPIO_WritePin(cfg->gpio_port_a, cfg->gpio_pin_a, GPIO_PIN_SET);
            HAL_GPIO_WritePin(cfg->gpio_port_b, cfg->gpio_pin_b, GPIO_PIN_SET);
            bdc_vnh_setSpeed(m, cfg->pwm_full_scale);
            break;

        default:
            return MOTOR_ERR_INVALID_PARAM;
    }
    return MOTOR_OK;
}

