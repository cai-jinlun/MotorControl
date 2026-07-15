/** 
  ******************************************************************************
  * @file    drv8714.c
  * @brief   DRV8714-Q1/S-Q1 智能半桥栅极驱动器控制实现
  * 
  * SPI 帧格式（基于 DRV871x 系列常见定义）：
  *   - 16 bit / MSB first
  *   - bit15: R/W (0=写, 1=读)
  *   - bit14-8: 寄存器地址 (7 bit)
  *   - bit7-0:  数据 (8 bit)
  * 
  * 返回帧：
  *   - bit15-8: IC_STAT1 状态字节
  *   - bit7-0:  读出数据（读操作）或上次写入数据/无关（写操作）
  * 
  * 注意：
  *   1. 当前 CubeMX 配置 SPI3 为 16-bit、CPOL=LOW、CPHA=1EDGE。
  *      若与 DRV8714 实际要求不符（常见为 CPHA=2EDGE），请在 CubeMX 中修改
  *      SPI3 的 Clock Phase 后重新生成代码。
  *   2. 本文件只涉及 SPI 读写与 PWM 控制，具体电机拓扑（H 桥/独立半桥）
  *      请通过 DRV8714_DefaultHBridgeConfig() 或自行配置寄存器实现。
  ******************************************************************************
  */
#include "drv8714.h"
#include "spi.h"
#include "tim.h"

/* SPI 命令掩码 */
#define DRV8714_SPI_READ_MSK      0x8000U
#define DRV8714_SPI_ADDR_SHIFT    8U
#define DRV8714_SPI_ADDR_MSK      0x7FU
#define DRV8714_SPI_DATA_MSK      0xFFU

/* 片选拉低/拉高 */
static inline void DRV8714_CS_Low(void)
{
    HAL_GPIO_WritePin(DRV8714_CS_PORT, DRV8714_CS_PIN, GPIO_PIN_RESET);
}

static inline void DRV8714_CS_High(void)
{
    HAL_GPIO_WritePin(DRV8714_CS_PORT, DRV8714_CS_PIN, GPIO_PIN_SET);
}

uint16_t DRV8714_SPI_Transceive(uint16_t tx_data)
{
    uint16_t rx_data = 0U;

    DRV8714_CS_Low();
    HAL_SPI_TransmitReceive(DRV8714_SPI_HANDLE, (uint8_t *)&tx_data,
                            (uint8_t *)&rx_data, 1U, 100U);
    DRV8714_CS_High();

    return rx_data;
}

uint8_t DRV8714_ReadReg(uint8_t addr)
{
    uint16_t tx = DRV8714_SPI_READ_MSK | (((uint16_t)(addr & DRV8714_SPI_ADDR_MSK)) << DRV8714_SPI_ADDR_SHIFT);
    uint16_t rx = DRV8714_SPI_Transceive(tx);
    return (uint8_t)(rx & DRV8714_SPI_DATA_MSK);
}

void DRV8714_WriteReg(uint8_t addr, uint8_t data)
{
    uint16_t tx = (((uint16_t)(addr & DRV8714_SPI_ADDR_MSK)) << DRV8714_SPI_ADDR_SHIFT)
                | ((uint16_t)(data & DRV8714_SPI_DATA_MSK));
    DRV8714_SPI_Transceive(tx);
}

uint8_t DRV8714_ReadStatus(void)
{
    /* 读任意寄存器时返回帧高 8 位即为 IC_STAT1 */
    uint16_t rx = DRV8714_SPI_Transceive(DRV8714_SPI_READ_MSK
                   | (((uint16_t)DRV8714_REG_IC_STAT1) << DRV8714_SPI_ADDR_SHIFT));
    return (uint8_t)(rx >> 8U);
}

void DRV8714_EnableDriver(void)
{
    uint8_t val = DRV8714_ReadReg(DRV8714_REG_IC_CTRL1);
    val |= IC_CTRL1_EN_DRV_Msk;
    DRV8714_WriteReg(DRV8714_REG_IC_CTRL1, val);
}

void DRV8714_DisableDriver(void)
{
    uint8_t val = DRV8714_ReadReg(DRV8714_REG_IC_CTRL1);
    val &= ~IC_CTRL1_EN_DRV_Msk;
    DRV8714_WriteReg(DRV8714_REG_IC_CTRL1, val);
}

void DRV8714_ClearFault(void)
{
    uint8_t val = DRV8714_ReadReg(DRV8714_REG_IC_CTRL1);
    val |= IC_CTRL1_CLR_FLT_Msk;
    DRV8714_WriteReg(DRV8714_REG_IC_CTRL1, val);

    /* 自清零位，可再读一次确认 */
    (void)DRV8714_ReadReg(DRV8714_REG_IC_CTRL1);
}

void DRV8714_SetHalfBridge(uint8_t hb, uint8_t mode)
{
    uint8_t pos;
    uint8_t val;

    if ((mode > BRG_CTRL_PWM) || (hb < DRV8714_HB1) || (hb > DRV8714_HB4))
    {
        return;
    }

    switch (hb)
    {
        case DRV8714_HB1: pos = BRG_CTRL1_HB1_Pos; break;
        case DRV8714_HB2: pos = BRG_CTRL1_HB2_Pos; break;
        case DRV8714_HB3: pos = BRG_CTRL1_HB3_Pos; break;
        case DRV8714_HB4: pos = BRG_CTRL1_HB4_Pos; break;
        default: return;
    }

    val = DRV8714_ReadReg(DRV8714_REG_BRG_CTRL1);
    val &= ~(0x3U << pos);
    val |= (mode << pos);
    DRV8714_WriteReg(DRV8714_REG_BRG_CTRL1, val);
}

void DRV8714_MapHalfBridgePWM(uint8_t hb, uint8_t pwm_in)
{
    uint8_t pos;
    uint8_t val;

    if ((pwm_in > PWM_MAP_IN4) || (hb < DRV8714_HB1) || (hb > DRV8714_HB4))
    {
        return;
    }

    switch (hb)
    {
        case DRV8714_HB1: pos = PWM_CTRL1_HB1_Pos; break;
        case DRV8714_HB2: pos = PWM_CTRL1_HB2_Pos; break;
        case DRV8714_HB3: pos = PWM_CTRL1_HB3_Pos; break;
        case DRV8714_HB4: pos = PWM_CTRL1_HB4_Pos; break;
        default: return;
    }

    val = DRV8714_ReadReg(DRV8714_REG_PWM_CTRL1);
    val &= ~(0x3U << pos);
    val |= (pwm_in << pos);
    DRV8714_WriteReg(DRV8714_REG_PWM_CTRL1, val);
}

void DRV8714_DefaultHBridgeConfig(void)
{
    uint8_t ctrl1;

    /* 确保 CS 初始高电平（CubeMX 已配置 PD0 初始 SET，此处再保险一次） */
    DRV8714_CS_High();

    /* 解锁寄存器（如需要），写入默认 LOCK 解锁值 */
    ctrl1 = DRV8714_ReadReg(DRV8714_REG_IC_CTRL1);
    ctrl1 &= ~IC_CTRL1_LOCK_Msk;
    ctrl1 |= (LOCK_UNLOCK << IC_CTRL1_LOCK_Pos);
    DRV8714_WriteReg(DRV8714_REG_IC_CTRL1, ctrl1);

    /* H 桥模式：HB1/HB2 组成一个 H 桥，HB3/HB4 组成另一个 H 桥 */
    ctrl1 = DRV8714_ReadReg(DRV8714_REG_IC_CTRL1);
    ctrl1 &= ~IC_CTRL1_BRG_MODE_Msk;
    ctrl1 |= (BRG_MODE_H_BRIDGE << IC_CTRL1_BRG_MODE_Pos);
    DRV8714_WriteReg(DRV8714_REG_IC_CTRL1, ctrl1);

    /* 
     * 默认只使能 HB1/HB2 用于第一路电机，由 PWM1(PB3=IN1) 和 PWM2(PB10=IN2) 控制。
     * HB3/HB4 置高阻，如需第二路电机请自行改为 PWM 模式并映射到可用 PWM。
     */
    DRV8714_WriteReg(DRV8714_REG_BRG_CTRL1,
                     (BRG_CTRL_PWM << BRG_CTRL1_HB1_Pos) |
                     (BRG_CTRL_PWM << BRG_CTRL1_HB2_Pos) |
                     (BRG_CTRL_HIZ << BRG_CTRL1_HB3_Pos) |
                     (BRG_CTRL_HIZ << BRG_CTRL1_HB4_Pos));

    /* PWM 映射：HB1->IN1(PWM1), HB2->IN2(PWM2)，其余半桥不映射 */
    DRV8714_WriteReg(DRV8714_REG_PWM_CTRL1,
                     (PWM_MAP_IN1 << PWM_CTRL1_HB1_Pos) |
                     (PWM_MAP_IN2 << PWM_CTRL1_HB2_Pos) |
                     (PWM_MAP_IN1 << PWM_CTRL1_HB3_Pos) |
                     (PWM_MAP_IN2 << PWM_CTRL1_HB4_Pos));

    /* 清故障并开启驱动 */
    DRV8714_ClearFault();
    DRV8714_EnableDriver();
}
