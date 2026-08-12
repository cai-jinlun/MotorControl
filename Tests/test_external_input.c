#include "external_input.h"
#include "external_input_port.h"

#include <assert.h>
#include <stdint.h>

static ExternalInputMask_t s_raw_high_mask;

uint32_t ExternalInput_PortReadHighMask(void)
{
    return s_raw_high_mask;
}

static void set_level(ExternalInputId_t id, uint8_t is_high)
{
    ExternalInputMask_t bit = EXTERNAL_INPUT_MASK(id);

    if (is_high != 0U) {
        s_raw_high_mask |= bit;
    } else {
        s_raw_high_mask &= ~bit;
    }
}

/* First call observes the transition; following calls prove its stable time. */
static void settle_level(ExternalInputId_t id,
                         uint8_t is_high,
                         uint32_t step_ms,
                         uint32_t stable_ms)
{
    uint32_t elapsed_ms;

    set_level(id, is_high);
    assert(ExternalInput_Service(step_ms) == EXTERNAL_INPUT_OK);
    for (elapsed_ms = 0U; elapsed_ms < stable_ms; elapsed_ms += step_ms) {
        assert(ExternalInput_Service(step_ms) == EXTERNAL_INPUT_OK);
    }
}

int main(void)
{
    ExternalInputMask_t rising;
    ExternalInputMask_t falling;
    uint8_t value;
    uint32_t index;

    assert(ExternalInput_Service(1U) ==
           EXTERNAL_INPUT_ERR_NOT_INITIALIZED);

    /* Start with CENTRAL and KEY3 active (LOW); initialization emits no edge. */
    s_raw_high_mask = EXTERNAL_INPUT_ALL_MASK;
    set_level(EXTERNAL_INPUT_CENTRAL_LOCK, 0U);
    set_level(EXTERNAL_INPUT_KEY3, 0U);
    assert(ExternalInput_Init() == EXTERNAL_INPUT_OK);
    assert(ExternalInput_Init() ==
           EXTERNAL_INPUT_ERR_ALREADY_INITIALIZED);
    assert(ExternalInput_IsActive(EXTERNAL_INPUT_CENTRAL_LOCK, &value) ==
           EXTERNAL_INPUT_OK);
    assert(value == 1U);
    assert(ExternalInput_IsActive(EXTERNAL_INPUT_KEY3, &value) ==
           EXTERNAL_INPUT_OK);
    assert(value == 1U);
    assert(ExternalInput_HasPendingEdges(&value) == EXTERNAL_INPUT_OK);
    assert(value == 0U);
    assert(ExternalInput_TakeEdges(&rising, &falling) == EXTERNAL_INPUT_OK);
    assert((rising == 0U) && (falling == 0U));

    /* Door-lock input: 10 ms debounce with electrical falling/rising edges. */
    set_level(EXTERNAL_INPUT_HALF_LOCK, 0U);
    assert(ExternalInput_Service(1U) == EXTERNAL_INPUT_OK);
    for (index = 0U; index < 9U; ++index) {
        assert(ExternalInput_Service(1U) == EXTERNAL_INPUT_OK);
    }
    assert(ExternalInput_GetLevel(EXTERNAL_INPUT_HALF_LOCK, &value) ==
           EXTERNAL_INPUT_OK);
    assert(value == 1U);
    assert(ExternalInput_Service(1U) == EXTERNAL_INPUT_OK);
    assert(ExternalInput_IsActive(EXTERNAL_INPUT_HALF_LOCK, &value) ==
           EXTERNAL_INPUT_OK);
    assert(value == 1U);
    assert(ExternalInput_TakeEdges(&rising, &falling) == EXTERNAL_INPUT_OK);
    assert(rising == 0U);
    assert(falling == EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_HALF_LOCK));

    settle_level(EXTERNAL_INPUT_HALF_LOCK, 1U, 1U, 10U);
    assert(ExternalInput_TakeEdges(&rising, &falling) == EXTERNAL_INPUT_OK);
    assert(rising == EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_HALF_LOCK));
    assert(falling == 0U);

    /* A pulse shorter than 10 ms must not change FULL or publish an edge. */
    set_level(EXTERNAL_INPUT_FULL_LOCK, 0U);
    assert(ExternalInput_Service(1U) == EXTERNAL_INPUT_OK);
    for (index = 0U; index < 5U; ++index) {
        assert(ExternalInput_Service(1U) == EXTERNAL_INPUT_OK);
    }
    set_level(EXTERNAL_INPUT_FULL_LOCK, 1U);
    assert(ExternalInput_Service(1U) == EXTERNAL_INPUT_OK);
    for (index = 0U; index < 20U; ++index) {
        assert(ExternalInput_Service(1U) == EXTERNAL_INPUT_OK);
    }
    assert(ExternalInput_GetLevel(EXTERNAL_INPUT_FULL_LOCK, &value) ==
           EXTERNAL_INPUT_OK);
    assert(value == 1U);
    assert(ExternalInput_HasPendingEdges(&value) == EXTERNAL_INPUT_OK);
    assert(value == 0U);

    /* A 5 ms producer period preserves 10/20 ms thresholds. */
    set_level(EXTERNAL_INPUT_FULL_LOCK, 0U);
    set_level(EXTERNAL_INPUT_KEY1, 0U);
    assert(ExternalInput_Service(5U) == EXTERNAL_INPUT_OK);
    for (index = 0U; index < 4U; ++index) {
        assert(ExternalInput_Service(5U) == EXTERNAL_INPUT_OK);
    }
    assert(ExternalInput_TakeEdges(&rising, &falling) == EXTERNAL_INPUT_OK);
    assert(rising == 0U);
    assert(falling == (EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_FULL_LOCK) |
                       EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_KEY1)));

    /* Both directions remain pending if the consumer is delayed. */
    settle_level(EXTERNAL_INPUT_KEY2, 0U, 5U, 20U);
    settle_level(EXTERNAL_INPUT_KEY2, 1U, 5U, 20U);
    assert(ExternalInput_HasPendingEdges(&value) == EXTERNAL_INPUT_OK);
    assert(value == 1U);
    assert(ExternalInput_TakeEdges(&rising, &falling) == EXTERNAL_INPUT_OK);
    assert(rising == EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_KEY2));
    assert(falling == EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_KEY2));
    assert(ExternalInput_HasPendingEdges(&value) == EXTERNAL_INPUT_OK);
    assert(value == 0U);

    assert(ExternalInput_Service(0U) ==
           EXTERNAL_INPUT_ERR_INVALID_PARAM);
    assert(ExternalInput_GetLevel(EXTERNAL_INPUT_COUNT, &value) ==
           EXTERNAL_INPUT_ERR_INVALID_PARAM);
    return 0;
}
