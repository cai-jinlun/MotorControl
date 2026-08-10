#ifndef __EXTERNAL_INPUT_PORT_H
#define __EXTERNAL_INPUT_PORT_H

#include <stdint.h>

/*
 * 板级适配接口：返回与 ExternalInputId_t bit 顺序一致的高电平位图。
 * 去抖核心只依赖该快照，不直接依赖 GPIO、TIM6 或 FreeRTOS。
 */
uint32_t ExternalInput_PortReadHighMask(void);

#endif /* __EXTERNAL_INPUT_PORT_H */
