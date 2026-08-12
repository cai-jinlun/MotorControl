#include "external_input.h"
#include "external_input_port.h"

#include "main.h"

/* 将板级 GPIO 电平映射为与 ExternalInputId_t 对应的高电平位图。 */
uint32_t ExternalInput_PortReadHighMask(void)
{
    ExternalInputMask_t high_mask = 0U;

    /* 每次调度构造一个完整位图快照，去抖核心不持有 HAL GPIO 细节。 */
    if (HAL_GPIO_ReadPin(HALF_GPIO_Port, HALF_Pin) == GPIO_PIN_SET) {
        high_mask |= EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_HALF_LOCK);
    }
    if (HAL_GPIO_ReadPin(FULL_GPIO_Port, FULL_Pin) == GPIO_PIN_SET) {
        high_mask |= EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_FULL_LOCK);
    }
    if (HAL_GPIO_ReadPin(CENTRAL_GPIO_Port, CENTRAL_Pin) == GPIO_PIN_SET) {
        high_mask |= EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_CENTRAL_LOCK);
    }
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET) {
        high_mask |= EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_KEY1);
    }
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_SET) {
        high_mask |= EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_KEY2);
    }
    if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_SET) {
        high_mask |= EXTERNAL_INPUT_MASK(EXTERNAL_INPUT_KEY3);
    }

    return high_mask;
}
