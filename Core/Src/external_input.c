#include "external_input.h"
#include "external_input_port.h"

#include <stddef.h>

/* 阈值使用真实毫秒而非调用次数，因此可兼容 1 ms ISR 和 5 ms RTOS 任务。 */
#define EXTERNAL_INPUT_LOCK_DEBOUNCE_MS 10U
#define EXTERNAL_INPUT_KEY_DEBOUNCE_MS  20U

typedef struct {
    uint32_t debounce_ms;           /* 当前通道要求的连续稳定时间 */
    uint32_t candidate_elapsed_ms;  /* 候选电平已经连续保持的时间 */
    uint8_t candidate_high;         /* 尚未通过去抖确认的原始电平 */
    /*
     * 仅生产者递增 generation，消费者只读 generation 并更新自己的游标。
     * Cortex-M4 对齐的 32 位读写是原子的，因此不需要 ISR/任务共同清位。
     */
    volatile uint32_t rising_generation;
    volatile uint32_t falling_generation;
    uint32_t consumed_rising_generation;
    uint32_t consumed_falling_generation;
} ExternalInputChannel_t;

static ExternalInputChannel_t s_channels[EXTERNAL_INPUT_COUNT];
/* 生产者写、任意观察者读；单个对齐的 32 位快照保证六路状态一致。 */
static volatile ExternalInputMask_t s_stable_high_mask;
static uint8_t s_initialized;

static uint8_t external_input_id_valid(ExternalInputId_t id)
{
    return ((uint32_t)id < (uint32_t)EXTERNAL_INPUT_COUNT) ? 1U : 0U;
}

static uint32_t external_input_debounce_ms(ExternalInputId_t id)
{
    return ((id == EXTERNAL_INPUT_HALF_LOCK) ||
            (id == EXTERNAL_INPUT_FULL_LOCK) ||
            (id == EXTERNAL_INPUT_CENTRAL_LOCK))
               ? EXTERNAL_INPUT_LOCK_DEBOUNCE_MS
               : EXTERNAL_INPUT_KEY_DEBOUNCE_MS;
}

static void external_input_publish_level(uint32_t index, uint8_t is_high)
{
    ExternalInputMask_t bit = EXTERNAL_INPUT_MASK(index);
    ExternalInputChannel_t *channel = &s_channels[index];

    if (is_high != 0U) {
        /* 先发布稳定状态，再递增事件代际，消费者看到事件时状态已同步。 */
        s_stable_high_mask |= bit;
        ++channel->rising_generation;
    } else {
        /* 下降沿同样保持“状态先于事件”发布顺序。 */
        s_stable_high_mask &= ~bit;
        ++channel->falling_generation;
    }
}

ExternalInputStatus_t ExternalInput_Init(void)
{
    ExternalInputMask_t raw_high_mask;
    uint32_t index;

    if (s_initialized != 0U) {
        return EXTERNAL_INPUT_ERR_ALREADY_INITIALIZED;
    }

    raw_high_mask = ExternalInput_PortReadHighMask() & EXTERNAL_INPUT_ALL_MASK;
    /* 初始电平直接作为稳定状态，避免上电时生成虚假的上升/下降沿。 */
    s_stable_high_mask = raw_high_mask;

    for (index = 0U; index < (uint32_t)EXTERNAL_INPUT_COUNT; ++index) {
        ExternalInputChannel_t *channel = &s_channels[index];

        channel->debounce_ms =
            external_input_debounce_ms((ExternalInputId_t)index);
        channel->candidate_elapsed_ms = 0U;
        channel->candidate_high =
            ((raw_high_mask & EXTERNAL_INPUT_MASK(index)) != 0U) ? 1U : 0U;
        channel->rising_generation = 0U;
        channel->falling_generation = 0U;
        channel->consumed_rising_generation = 0U;
        channel->consumed_falling_generation = 0U;
    }

    s_initialized = 1U;
    return EXTERNAL_INPUT_OK;
}

ExternalInputStatus_t ExternalInput_Service(uint32_t elapsed_ms)
{
    ExternalInputMask_t raw_high_mask;
    uint32_t index;

    if (s_initialized == 0U) {
        return EXTERNAL_INPUT_ERR_NOT_INITIALIZED;
    }
    if (elapsed_ms == 0U) {
        return EXTERNAL_INPUT_ERR_INVALID_PARAM;
    }

    raw_high_mask = ExternalInput_PortReadHighMask() & EXTERNAL_INPUT_ALL_MASK;

    for (index = 0U; index < (uint32_t)EXTERNAL_INPUT_COUNT; ++index) {
        ExternalInputChannel_t *channel = &s_channels[index];
        ExternalInputMask_t bit = EXTERNAL_INPUT_MASK(index);
        uint8_t raw_high = ((raw_high_mask & bit) != 0U) ? 1U : 0U;
        uint8_t stable_high = ((s_stable_high_mask & bit) != 0U) ? 1U : 0U;
        uint32_t remaining_ms;

        if (raw_high != channel->candidate_high) {
            /* 首次观察到新候选值，只建立候选；不推测两次采样之间的稳定时间。 */
            channel->candidate_high = raw_high;
            channel->candidate_elapsed_ms = 0U;
            continue;
        }

        if (raw_high == stable_high) {
            /* 原始值已回到稳定值，之前尚未确认的抖动立即作废。 */
            channel->candidate_elapsed_ms = 0U;
            continue;
        }

        remaining_ms = channel->debounce_ms -
                       channel->candidate_elapsed_ms;
        /* elapsed_ms 可大于剩余时间，比较写法同时避免计数累加溢出。 */
        if (elapsed_ms >= remaining_ms) {
            channel->candidate_elapsed_ms = 0U;
            external_input_publish_level(index, raw_high);
        } else {
            channel->candidate_elapsed_ms += elapsed_ms;
        }
    }

    return EXTERNAL_INPUT_OK;
}

ExternalInputStatus_t ExternalInput_GetLevel(ExternalInputId_t id,
                                             uint8_t *is_high)
{
    ExternalInputMask_t stable_high_mask;

    if (is_high == NULL) return EXTERNAL_INPUT_ERR_NULL_PTR;
    if (s_initialized == 0U) return EXTERNAL_INPUT_ERR_NOT_INITIALIZED;
    if (external_input_id_valid(id) == 0U) {
        return EXTERNAL_INPUT_ERR_INVALID_PARAM;
    }

    stable_high_mask = s_stable_high_mask;
    *is_high = ((stable_high_mask & EXTERNAL_INPUT_MASK(id)) != 0U) ? 1U : 0U;
    return EXTERNAL_INPUT_OK;
}

ExternalInputStatus_t ExternalInput_IsActive(ExternalInputId_t id,
                                             uint8_t *is_active)
{
    uint8_t is_high;
    ExternalInputStatus_t status;

    if (is_active == NULL) return EXTERNAL_INPUT_ERR_NULL_PTR;

    status = ExternalInput_GetLevel(id, &is_high);
    if (status != EXTERNAL_INPUT_OK) {
        return status;
    }

    *is_active = (is_high == 0U) ? 1U : 0U;
    return EXTERNAL_INPUT_OK;
}

ExternalInputStatus_t ExternalInput_TakeEdges(
    ExternalInputMask_t *rising_mask,
    ExternalInputMask_t *falling_mask)
{
    ExternalInputMask_t rising = 0U;
    ExternalInputMask_t falling = 0U;
    uint32_t index;

    if ((rising_mask == NULL) || (falling_mask == NULL)) {
        return EXTERNAL_INPUT_ERR_NULL_PTR;
    }
    if (s_initialized == 0U) {
        return EXTERNAL_INPUT_ERR_NOT_INITIALIZED;
    }

    for (index = 0U; index < (uint32_t)EXTERNAL_INPUT_COUNT; ++index) {
        ExternalInputChannel_t *channel = &s_channels[index];
        uint32_t rising_generation = channel->rising_generation;
        uint32_t falling_generation = channel->falling_generation;

        /*
         * 先读取生产者代际，再推进消费者游标。若此后 ISR/任务递增代际，
         * 新值仍与游标不同，会自然留到下一次 TakeEdges，因而无需共同清位。
         */
        if (rising_generation != channel->consumed_rising_generation) {
            rising |= EXTERNAL_INPUT_MASK(index);
            channel->consumed_rising_generation = rising_generation;
        }
        if (falling_generation != channel->consumed_falling_generation) {
            falling |= EXTERNAL_INPUT_MASK(index);
            channel->consumed_falling_generation = falling_generation;
        }
    }

    *rising_mask = rising;
    *falling_mask = falling;
    return EXTERNAL_INPUT_OK;
}

ExternalInputStatus_t ExternalInput_HasPendingEdges(uint8_t *has_event)
{
    uint32_t index;

    if (has_event == NULL) return EXTERNAL_INPUT_ERR_NULL_PTR;
    if (s_initialized == 0U) {
        return EXTERNAL_INPUT_ERR_NOT_INITIALIZED;
    }

    /* 只比较生产者代际与消费者游标，不改变任何消费状态。 */
    *has_event = 0U;
    for (index = 0U; index < (uint32_t)EXTERNAL_INPUT_COUNT; ++index) {
        const ExternalInputChannel_t *channel = &s_channels[index];

        if ((channel->rising_generation !=
             channel->consumed_rising_generation) ||
            (channel->falling_generation !=
             channel->consumed_falling_generation)) {
            *has_event = 1U;
            break;
        }
    }

    return EXTERNAL_INPUT_OK;
}
