#include "board_motor_link.h"

#include "current_sense.h"
#include "drv8714.h"
#include "main.h"
#include "motor_bdc_drv.h"
#include "motor_bdc_vnh.h"
#include "quad_encoder.h"
#include "tim.h"

#include <stddef.h>

#define DOOR_CURRENT_OFFSET_COUNTS  2048.0f
#define DOOR_CURRENT_AMPS_PER_COUNT 0.0201f
#define BOARD_MOTOR_DEAD_ZONE        0U
/* 阈值 0 表示上电默认禁用，必须在机构标定后由上层显式配置。 */
#define DOOR_RUN_TIMEOUT_MS          0U
#define UNLOCK_RUN_TIMEOUT_MS        0U
#define CINCH_RUN_TIMEOUT_MS         0U

typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t           channel_a;
    uint32_t           channel_b;
} DoorMotorContext_t;

typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t           channel;
    GPIO_TypeDef      *in_a_port;
    uint16_t           in_a_pin;
    GPIO_TypeDef      *in_b_port;
    uint16_t           in_b_pin;
} VnhMotorContext_t;

static MotorErr_t door_drv_port_init(void *context);
static MotorErr_t door_drv_port_deinit(void *context);
static MotorErr_t door_drv_set_inputs(void *context, uint16_t in1, uint16_t in2);
static MotorErr_t door_drv_get_position(void *context, int32_t *position);
static MotorErr_t door_drv_reset_position(void *context, int32_t position);
static MotorErr_t door_drv_get_velocity(void *context, float *velocity);
static MotorErr_t door_drv_get_current(void *context, float *current);

static MotorErr_t lock_vnh_port_init(void *context);
static MotorErr_t lock_vnh_port_deinit(void *context);
static MotorErr_t lock_vnh_set_outputs(void *context,
                                       uint8_t in_a,
                                       uint8_t in_b,
                                       uint16_t pwm);

static const MotorBDC_DRV_PortOps_t s_door_drv_port_ops = {
    .init           = door_drv_port_init,
    .deinit         = door_drv_port_deinit,
    .set_inputs     = door_drv_set_inputs,
    .get_position   = door_drv_get_position,
    .reset_position = door_drv_reset_position,
    .get_velocity   = door_drv_get_velocity,
    .get_current    = door_drv_get_current,
};

static const MotorBDC_VNH_PortOps_t s_lock_vnh_port_ops = {
    .init           = lock_vnh_port_init,
    .deinit         = lock_vnh_port_deinit,
    .set_outputs    = lock_vnh_set_outputs,
    .get_position   = NULL,
    .reset_position = NULL,
    .get_velocity   = NULL,
    .get_current    = NULL,
};

static DoorMotorContext_t s_door_context = {
    .timer     = &htim2,
    .channel_a = TIM_CHANNEL_2, /* H1_PWM1 / DRV8714 IN1 */
    .channel_b = TIM_CHANNEL_3, /* H1_PWM2 / DRV8714 IN2 */
};

static VnhMotorContext_t s_unlock_context = {
    .timer     = &htim4,
    .channel   = TIM_CHANNEL_1,
    .in_a_port = H4_INA_GPIO_Port,
    .in_a_pin  = H4_INA_Pin,
    .in_b_port = H4_INB_GPIO_Port,
    .in_b_pin  = H4_INB_Pin,
};

static VnhMotorContext_t s_cinch_context = {
    .timer     = &htim4,
    .channel   = TIM_CHANNEL_2,
    .in_a_port = H5_INA_GPIO_Port,
    .in_a_pin  = H5_INA_Pin,
    .in_b_port = H5_INB_GPIO_Port,
    .in_b_pin  = H5_INB_Pin,
};

static MotorHandle_t *s_door_motor;
static MotorHandle_t *s_unlock_motor;
static MotorHandle_t *s_cinch_motor;
static uint8_t s_initialized;

/*
 * 将通用电机输出 0~1000 映射到定时器比较值。
 * PWM 模式 1 需要 CCR > ARR 才能保持 100% 占空比，因此按 ARR + 1 个计数刻度换算。
 */
static uint32_t output_to_compare(const TIM_HandleTypeDef *timer,
                                  uint16_t output)
{
    uint64_t period_counts;
    uint64_t compare;

    if (output > MOTOR_OUTPUT_MAX) {
        output = MOTOR_OUTPUT_MAX;
    }

    period_counts = (uint64_t)__HAL_TIM_GET_AUTORELOAD(timer) + 1U;
    compare = (((uint64_t)output * period_counts) +
               (MOTOR_OUTPUT_MAX / 2U)) /
              MOTOR_OUTPUT_MAX;

    return (compare > UINT32_MAX) ? UINT32_MAX : (uint32_t)compare;
}

static MotorErr_t current_status_to_motor_error(CurrentSenseStatus_t status)
{
    switch (status) {
    case CURRENT_SENSE_OK:
        return MOTOR_OK;
    case CURRENT_SENSE_ERR_NULL_PTR:
        return MOTOR_ERR_NULL_PTR;
    case CURRENT_SENSE_ERR_NOT_STARTED:
    case CURRENT_SENSE_ERR_NOT_CALIBRATED:
        return MOTOR_ERR_NOT_INITIALIZED;
    case CURRENT_SENSE_ERR_NOT_READY:
        return MOTOR_ERR_NOT_READY;
    case CURRENT_SENSE_ERR_INVALID_CALIBRATION:
        return MOTOR_ERR_INVALID_PARAM;
    case CURRENT_SENSE_ERR_HW_FAILURE:
    default:
        return MOTOR_ERR_HW_FAILURE;
    }
}

static MotorErr_t door_drv_port_init(void *context)
{
    DoorMotorContext_t *door = (DoorMotorContext_t *)context;
    CurrentSenseStatus_t current_status;

    if ((door == NULL) || (door->timer == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }

    /* 先提交零占空比，再启动 PWM，避免使能 DRV8714 时产生意外脉冲。 */
    __HAL_TIM_SET_COMPARE(door->timer, door->channel_a, 0U);
    __HAL_TIM_SET_COMPARE(door->timer, door->channel_b, 0U);
    if (HAL_TIM_PWM_Start(door->timer, door->channel_a) != HAL_OK) {
        return MOTOR_ERR_HW_FAILURE;
    }
    if (HAL_TIM_PWM_Start(door->timer, door->channel_b) != HAL_OK) {
        (void)HAL_TIM_PWM_Stop(door->timer, door->channel_a);
        return MOTOR_ERR_HW_FAILURE;
    }

    DRV8714_DefaultHBridgeConfig();
    QuadEncoder_Init();

    current_status = CurrentSense_SetCalibration(
        DOOR_CURRENT_OFFSET_COUNTS,
        DOOR_CURRENT_AMPS_PER_COUNT);
    if (current_status == CURRENT_SENSE_OK) {
        current_status = CurrentSense_Start();
    }
    if (current_status != CURRENT_SENSE_OK) {
        DRV8714_DisableDriver();
        (void)HAL_TIM_PWM_Stop(door->timer, door->channel_b);
        (void)HAL_TIM_PWM_Stop(door->timer, door->channel_a);
        return current_status_to_motor_error(current_status);
    }

    return MOTOR_OK;
}

static MotorErr_t door_drv_port_deinit(void *context)
{
    DoorMotorContext_t *door = (DoorMotorContext_t *)context;
    HAL_StatusTypeDef status_a;
    HAL_StatusTypeDef status_b;
    CurrentSenseStatus_t current_status;

    if ((door == NULL) || (door->timer == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }

    __HAL_TIM_SET_COMPARE(door->timer, door->channel_a, 0U);
    __HAL_TIM_SET_COMPARE(door->timer, door->channel_b, 0U);
    DRV8714_DisableDriver();
    current_status = CurrentSense_Stop();
    status_b = HAL_TIM_PWM_Stop(door->timer, door->channel_b);
    status_a = HAL_TIM_PWM_Stop(door->timer, door->channel_a);

    if ((current_status != CURRENT_SENSE_OK) &&
        (current_status != CURRENT_SENSE_ERR_NOT_STARTED)) {
        return current_status_to_motor_error(current_status);
    }
    return ((status_a == HAL_OK) && (status_b == HAL_OK))
               ? MOTOR_OK
               : MOTOR_ERR_HW_FAILURE;
}

static MotorErr_t door_drv_set_inputs(void *context, uint16_t in1, uint16_t in2)
{
    DoorMotorContext_t *door = (DoorMotorContext_t *)context;

    if ((door == NULL) || (door->timer == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    if ((in1 > MOTOR_OUTPUT_MAX) || (in2 > MOTOR_OUTPUT_MAX)) {
        return MOTOR_ERR_INVALID_PARAM;
    }

    /* 换向前先清零双路比较值，降低瞬时桥臂冲突风险。 */
    __HAL_TIM_SET_COMPARE(door->timer, door->channel_a, 0U);
    __HAL_TIM_SET_COMPARE(door->timer, door->channel_b, 0U);
    __HAL_TIM_SET_COMPARE(door->timer,
                          door->channel_a,
                          output_to_compare(door->timer, in1));
    __HAL_TIM_SET_COMPARE(door->timer,
                          door->channel_b,
                          output_to_compare(door->timer, in2));
    return MOTOR_OK;
}

static MotorErr_t door_drv_get_position(void *context, int32_t *position)
{
    (void)context;
    if (position == NULL) {
        return MOTOR_ERR_NULL_PTR;
    }
    *position = QuadEncoder_GetCount();
    return MOTOR_OK;
}

static MotorErr_t door_drv_reset_position(void *context, int32_t position)
{
    (void)context;
    QuadEncoder_SetCount(position);
    return MOTOR_OK;
}

static MotorErr_t door_drv_get_velocity(void *context, float *velocity)
{
    float magnitude;
    int8_t direction;

    (void)context;
    if (velocity == NULL) {
        return MOTOR_ERR_NULL_PTR;
    }

    magnitude = QuadEncoder_GetVelocity();
    direction = QuadEncoder_GetDirection();
    *velocity = (direction < 0) ? -magnitude : magnitude;
    return MOTOR_OK;
}

static MotorErr_t door_drv_get_current(void *context, float *current)
{
    (void)context;
    /* 板级电机接口只上传已换算为安培的平均电流。 */
    return current_status_to_motor_error(CurrentSense_GetCurrent(current));
}

static MotorErr_t lock_vnh_port_init(void *context)
{
    VnhMotorContext_t *vnh = (VnhMotorContext_t *)context;

    if ((vnh == NULL) || (vnh->timer == NULL) ||
        (vnh->in_a_port == NULL) || (vnh->in_b_port == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }

    __HAL_TIM_SET_COMPARE(vnh->timer, vnh->channel, 0U);
    HAL_GPIO_WritePin(vnh->in_a_port, vnh->in_a_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(vnh->in_b_port, vnh->in_b_pin, GPIO_PIN_RESET);
    return (HAL_TIM_PWM_Start(vnh->timer, vnh->channel) == HAL_OK)
               ? MOTOR_OK
               : MOTOR_ERR_HW_FAILURE;
}

static MotorErr_t lock_vnh_port_deinit(void *context)
{
    VnhMotorContext_t *vnh = (VnhMotorContext_t *)context;

    if ((vnh == NULL) || (vnh->timer == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }

    __HAL_TIM_SET_COMPARE(vnh->timer, vnh->channel, 0U);
    HAL_GPIO_WritePin(vnh->in_a_port, vnh->in_a_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(vnh->in_b_port, vnh->in_b_pin, GPIO_PIN_RESET);
    return (HAL_TIM_PWM_Stop(vnh->timer, vnh->channel) == HAL_OK)
               ? MOTOR_OK
               : MOTOR_ERR_HW_FAILURE;
}

static MotorErr_t lock_vnh_set_outputs(void *context,
                                       uint8_t in_a,
                                       uint8_t in_b,
                                       uint16_t pwm)
{
    VnhMotorContext_t *vnh = (VnhMotorContext_t *)context;

    if ((vnh == NULL) || (vnh->timer == NULL)) {
        return MOTOR_ERR_NULL_PTR;
    }
    if ((in_a > 1U) || (in_b > 1U) || (pwm > MOTOR_OUTPUT_MAX)) {
        return MOTOR_ERR_INVALID_PARAM;
    }

    /* 切换 INA/INB 时先关闭 PWM，随后再恢复目标占空比。 */
    __HAL_TIM_SET_COMPARE(vnh->timer, vnh->channel, 0U);
    HAL_GPIO_WritePin(vnh->in_a_port,
                      vnh->in_a_pin,
                      (in_a != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(vnh->in_b_port,
                      vnh->in_b_pin,
                      (in_b != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(vnh->timer,
                          vnh->channel,
                          output_to_compare(vnh->timer, pwm));
    return MOTOR_OK;
}

/* 逆序销毁已创建的实例；失败的句柄保留以便下次重试。 */
static MotorErr_t board_motor_link_cleanup_instances(void)
{
    MotorErr_t first_error = MOTOR_OK;
    MotorErr_t status;

    if (s_cinch_motor != NULL) {
        status = MotorBDC_VNH_Destroy(s_cinch_motor);
        if (status == MOTOR_OK) {
            s_cinch_motor = NULL;
        } else if (first_error == MOTOR_OK) {
            first_error = status;
        }
    }

    if (s_unlock_motor != NULL) {
        status = MotorBDC_VNH_Destroy(s_unlock_motor);
        if (status == MOTOR_OK) {
            s_unlock_motor = NULL;
        } else if (first_error == MOTOR_OK) {
            first_error = status;
        }
    }

    if (s_door_motor != NULL) {
        status = MotorBDC_DRV_Destroy(s_door_motor);
        if (status == MOTOR_OK) {
            s_door_motor = NULL;
        } else if (first_error == MOTOR_OK) {
            first_error = status;
        }
    }

    return first_error;
}

MotorErr_t BoardMotorLink_Init(void)
{
    MotorErr_t cleanup_status;
    MotorErr_t status;
    MotorBDC_DRV_Config_t door_config;
    MotorBDC_VNH_Config_t unlock_config;
    MotorBDC_VNH_Config_t cinch_config;

    if (s_initialized != 0U) {
        return MOTOR_ERR_ALREADY_INIT;
    }

    /* 先重试收敛上次初始化失败遗留的实例，禁止覆盖有效句柄。 */
    cleanup_status = board_motor_link_cleanup_instances();
    if (cleanup_status != MOTOR_OK) {
        return cleanup_status;
    }

    /* 当前 HAL tick 由独立 TIM7 提供；引入 RTOS 后可只替换此时间源。 */
    status = Motor_SetTimeSource(HAL_GetTick);
    if (status != MOTOR_OK) {
        return status;
    }

    door_config.port = &s_door_drv_port_ops;
    door_config.context = &s_door_context;
    door_config.dead_zone = BOARD_MOTOR_DEAD_ZONE;
    s_door_motor = MotorBDC_DRV_Create(&door_config, &status);
    if (s_door_motor == NULL) {
        return status;
    }
    status = Motor_ConfigureRunTimeout(s_door_motor,
                                       DOOR_RUN_TIMEOUT_MS,
                                       /* 撑杆超时后主动制动，减少门体惯性移动。 */
                                       MOTOR_DIR_BRAKE);
    if (status != MOTOR_OK) {
        cleanup_status = board_motor_link_cleanup_instances();
        return (cleanup_status != MOTOR_OK) ? cleanup_status : status;
    }

    unlock_config.port = &s_lock_vnh_port_ops;
    unlock_config.context = &s_unlock_context;
    unlock_config.dead_zone = BOARD_MOTOR_DEAD_ZONE;
    s_unlock_motor = MotorBDC_VNH_Create(&unlock_config, &status);
    if (s_unlock_motor == NULL) {
        cleanup_status = board_motor_link_cleanup_instances();
        return (cleanup_status != MOTOR_OK) ? cleanup_status : status;
    }
    status = Motor_ConfigureRunTimeout(s_unlock_motor,
                                       UNLOCK_RUN_TIMEOUT_MS,
                                       /* 门锁电机使用自由滑行，避免持续制动发热。 */
                                       MOTOR_DIR_COAST);
    if (status != MOTOR_OK) {
        cleanup_status = board_motor_link_cleanup_instances();
        return (cleanup_status != MOTOR_OK) ? cleanup_status : status;
    }

    cinch_config.port = &s_lock_vnh_port_ops;
    cinch_config.context = &s_cinch_context;
    cinch_config.dead_zone = BOARD_MOTOR_DEAD_ZONE;
    s_cinch_motor = MotorBDC_VNH_Create(&cinch_config, &status);
    if (s_cinch_motor == NULL) {
        cleanup_status = board_motor_link_cleanup_instances();
        return (cleanup_status != MOTOR_OK) ? cleanup_status : status;
    }
    status = Motor_ConfigureRunTimeout(s_cinch_motor,
                                       CINCH_RUN_TIMEOUT_MS,
                                       /* 门锁电机使用自由滑行，避免持续制动发热。 */
                                       MOTOR_DIR_COAST);
    if (status != MOTOR_OK) {
        cleanup_status = board_motor_link_cleanup_instances();
        return (cleanup_status != MOTOR_OK) ? cleanup_status : status;
    }

    s_initialized = 1U;
    return MOTOR_OK;
}

MotorErr_t BoardMotorLink_Service(void)
{
    MotorErr_t first_error = MOTOR_OK;
    MotorErr_t status;

    if ((s_initialized == 0U) || (s_door_motor == NULL) ||
        (s_unlock_motor == NULL) || (s_cinch_motor == NULL)) {
        return MOTOR_ERR_NOT_INITIALIZED;
    }

    /* 即使某一电机服务失败，也继续检查其余电机，最后返回首个错误。 */
    status = Motor_Service(s_door_motor);
    if (status != MOTOR_OK) {
        first_error = status;
    }

    status = Motor_Service(s_unlock_motor);
    if ((status != MOTOR_OK) && (first_error == MOTOR_OK)) {
        first_error = status;
    }

    status = Motor_Service(s_cinch_motor);
    if ((status != MOTOR_OK) && (first_error == MOTOR_OK)) {
        first_error = status;
    }

    return first_error;
}

MotorHandle_t *BoardMotorLink_GetDoorMotor(void)
{
    return s_door_motor;
}

MotorHandle_t *BoardMotorLink_GetUnlockMotor(void)
{
    return s_unlock_motor;
}

MotorHandle_t *BoardMotorLink_GetCinchMotor(void)
{
    return s_cinch_motor;
}
