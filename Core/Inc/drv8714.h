/** 
  ******************************************************************************
  * @file    drv8714.h
  * @brief   DRV8714-Q1/S-Q1 智能半桥栅极驱动器控制接口
  * 
  * 硬件连接（基于当前 CubeMX 配置）：
  *   - SPI3:  PC10=SCK, PC11=SDO(MISO), PC12=SDI(MOSI), PD0=nSCS(H_CS)
  *   - PWM1: PB3  = TIM2_CH2 (H1_PWM1)
  *   - PWM2: PB10 = TIM2_CH3 (H1_PWM2)
  ******************************************************************************
  */
#ifndef __DRV8714_H
#define __DRV8714_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* SPI 与外设句柄 */
#define DRV8714_SPI_HANDLE        (&hspi3)
#define DRV8714_CS_PORT           H_CS_GPIO_Port
#define DRV8714_CS_PIN            H_CS_Pin

/* PWM 定时器与通道 */
#define DRV8714_PWM_TIM           (&htim2)
#define DRV8714_PWM_CH1           TIM_CHANNEL_2   /* PB3  H1_PWM1 */
#define DRV8714_PWM_CH2           TIM_CHANNEL_3   /* PB10 H1_PWM2 */
#define DRV8714_PWM_MAX_DUTY      1000U           /* 与 TIM2 Period 对应 */

/* =============== 寄存器地址 =============== */
#define DRV8714_REG_IC_STAT1      0x00U
#define DRV8714_REG_VDS_STAT1     0x01U
#define DRV8714_REG_VDS_STAT2     0x02U
#define DRV8714_REG_VGS_STAT1     0x03U
#define DRV8714_REG_VGS_STAT2     0x04U
#define DRV8714_REG_IC_STAT2      0x05U
#define DRV8714_REG_IC_STAT3      0x06U
#define DRV8714_REG_IC_CTRL1      0x07U
#define DRV8714_REG_IC_CTRL2      0x08U
#define DRV8714_REG_BRG_CTRL1     0x09U
#define DRV8714_REG_BRG_CTRL2     0x0AU
#define DRV8714_REG_PWM_CTRL1     0x0BU
#define DRV8714_REG_PWM_CTRL2     0x0CU
#define DRV8714_REG_PWM_CTRL3     0x0DU
#define DRV8714_REG_PWM_CTRL4     0x0EU
#define DRV8714_REG_IDRV_CTRL1    0x0FU
#define DRV8714_REG_IDRV_CTRL2    0x10U
#define DRV8714_REG_IDRV_CTRL3    0x11U
#define DRV8714_REG_IDRV_CTRL4    0x12U
#define DRV8714_REG_DRV_CTRL1     0x17U
#define DRV8714_REG_DRV_CTRL2     0x18U
#define DRV8714_REG_DRV_CTRL3     0x19U
#define DRV8714_REG_DRV_CTRL4     0x1AU
#define DRV8714_REG_VDS_CTRL1     0x1EU
#define DRV8714_REG_VDS_CTRL2     0x1FU
#define DRV8714_REG_OLSC_CTRL1    0x22U
#define DRV8714_REG_UVOV_CTRL     0x24U
#define DRV8714_REG_CSA_CTRL1     0x25U

/* =============== IC_CTRL1 (0x07) 位定义 =============== */
#define IC_CTRL1_EN_DRV_Pos       7U
#define IC_CTRL1_EN_DRV_Msk       (1U << IC_CTRL1_EN_DRV_Pos)
#define IC_CTRL1_EN_OLSC_Pos      6U
#define IC_CTRL1_EN_OLSC_Msk      (1U << IC_CTRL1_EN_OLSC_Pos)
#define IC_CTRL1_BRG_MODE_Pos     4U
#define IC_CTRL1_BRG_MODE_Msk     (0x3U << IC_CTRL1_BRG_MODE_Pos)
#define IC_CTRL1_LOCK_Pos         1U
#define IC_CTRL1_LOCK_Msk         (0x7U << IC_CTRL1_LOCK_Pos)
#define IC_CTRL1_CLR_FLT_Pos      0U
#define IC_CTRL1_CLR_FLT_Msk      (1U << IC_CTRL1_CLR_FLT_Pos)

/* BRG_MODE 模式 */
#define BRG_MODE_INDEPENDENT      0x0U  /* 独立半桥 */
#define BRG_MODE_H_BRIDGE         0x1U  /* H 桥模式 */
#define BRG_MODE_SOLENOID         0x3U  /* 分离式 HS/LS 电磁阀模式 */

/* LOCK 值（写入寄存器前通常需解锁，请对照最新数据手册确认） */
#define LOCK_UNLOCK               0x6U  /* 110b：解锁 */
#define LOCK_NORMAL               0x3U  /* 011b：锁定 */

/* =============== BRG_CTRL1 (0x09) 半桥控制 =============== */
#define BRG_CTRL_HIZ              0x0U  /* 高阻 */
#define BRG_CTRL_LS               0x1U  /* 仅低侧导通 */
#define BRG_CTRL_HS               0x2U  /* 仅高侧导通 */
#define BRG_CTRL_PWM              0x3U  /* PWM 控制 */

#define BRG_CTRL1_HB1_Pos         6U
#define BRG_CTRL1_HB2_Pos         4U
#define BRG_CTRL1_HB3_Pos         2U
#define BRG_CTRL1_HB4_Pos         0U

/* =============== PWM_CTRL1 (0x0B) PWM 输入映射 =============== */
#define PWM_CTRL1_HB1_Pos         6U
#define PWM_CTRL1_HB2_Pos         4U
#define PWM_CTRL1_HB3_Pos         2U
#define PWM_CTRL1_HB4_Pos         0U

#define PWM_MAP_IN1               0x0U
#define PWM_MAP_IN2               0x1U
#define PWM_MAP_IN3               0x2U
#define PWM_MAP_IN4               0x3U

/* =============== IC_STAT1 (0x00) 状态位 =============== */
#define IC_STAT1_SPI_OK           (1U << 7U)
#define IC_STAT1_POR              (1U << 6U)
#define IC_STAT1_FAULT            (1U << 5U)
#define IC_STAT1_WARN             (1U << 4U)
#define IC_STAT1_DS_GS            (1U << 3U)
#define IC_STAT1_UV               (1U << 2U)
#define IC_STAT1_OV               (1U << 1U)
#define IC_STAT1_OT_WD_AGD        (1U << 0U)

/* =============== IC_STAT2 (0x05) 状态位 =============== */
#define IC_STAT2_PVDD_UV          (1U << 7U)
#define IC_STAT2_PVDD_OV          (1U << 6U)
#define IC_STAT2_VCP_UV           (1U << 5U)
#define IC_STAT2_OTW              (1U << 4U)
#define IC_STAT2_OTSD             (1U << 3U)
#define IC_STAT2_WD_FLT           (1U << 2U)
#define IC_STAT2_SCLK_FLT         (1U << 1U)

/* =============== PWM_CTRL2/3/4 快速定义 =============== */
#define PWM_CTRL2_IN1_MODE_Pos    6U
#define PWM_CTRL2_IN2_MODE_Pos    4U
#define PWM_CTRL2_IN3_MODE_Pos    0U  /* 请按实际数据手册核对 */
#define PWM_CTRL2_IN4_MODE_Pos    2U  /* 请按实际数据手册核对 */

/* 半桥编号 */
#define DRV8714_HB1               1U
#define DRV8714_HB2               2U
#define DRV8714_HB3               3U
#define DRV8714_HB4               4U

/* =============== 接口函数 =============== */

/* 初始化：启动两路 PWM 并设置初始占空比为 0 */
void DRV8714_Init(void);

/* SPI 原始读写。返回的 16bit 中：[15:8]=状态字节，[7:0]=数据字节 */
uint16_t DRV8714_SPI_Transceive(uint16_t tx_data);

/* 读/写 8 位寄存器 */
uint8_t  DRV8714_ReadReg(uint8_t addr);
void     DRV8714_WriteReg(uint8_t addr, uint8_t data);

/* 读取 IC_STAT1 状态字节 */
uint8_t  DRV8714_ReadStatus(void);

/* 驱动器使能/禁能、清故障 */
void DRV8714_EnableDriver(void);
void DRV8714_DisableDriver(void);
void DRV8714_ClearFault(void);

/* 设置 PWM 占空比：duty 范围 0 ~ DRV8714_PWM_MAX_DUTY */
void DRV8714_SetPWM1(uint16_t duty);
void DRV8714_SetPWM2(uint16_t duty);

/* 设置单个半桥控制模式：BRG_CTRL_HIZ/LS/HS/PWM */
void DRV8714_SetHalfBridge(uint8_t hb, uint8_t mode);

/* 将半桥映射到 PWM 输入：PWM_MAP_IN1/IN2/IN3/IN4 */
void DRV8714_MapHalfBridgePWM(uint8_t hb, uint8_t pwm_in);

/* 默认配置示例：H 桥模式，HB1/HB2 由 IN1/IN2 控制，HB3/HB4 由 IN3/IN4 控制 */
void DRV8714_DefaultHBridgeConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV8714_H */
