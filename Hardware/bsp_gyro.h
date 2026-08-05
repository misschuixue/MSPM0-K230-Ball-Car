/**
 * @file bsp_gyro.h
 * @brief 六轴陀螺仪姿态感知驱动抽象层 (JY61P IMU BSP)
 * @note 负责软件模拟 I2C 协议、姿态角/角速度解算及连续航向积分。
 */

#ifndef __BSP_GYRO_H__
#define __BSP_GYRO_H__

/* ==================== 包含文件 (Includes) ==================== */
#include "main.h"

/* ==================== 硬件通信与寄存器配置宏 (Hardware Macros) ==================== */
#define GYRO_DEBUG 0
#define IIC_ADDR 0x50
#define YAW_REG_ADDR 0x3F
#define UN_REG 0x69
#define SAVE_REG 0x00
#define ANGLE_REFER_REG 0x01

/* ==================== 软件模拟 I2C 引脚操作宏 (Bit-Banging GPIO) ==================== */
#define SDA_IN()                                                               \
  { DL_GPIO_initDigitalInput(IIC_Software_JYSDA_IOMUX); }
#define SDA_OUT()                                                              \
  {                                                                            \
    DL_GPIO_initDigitalOutput(IIC_Software_JYSDA_IOMUX);                       \
    DL_GPIO_enableOutput(IIC_Software_PORT, IIC_Software_JYSDA_PIN);           \
  }
#define SCL(BIT)                                                               \
  (BIT ? DL_GPIO_setPins(IIC_Software_PORT, IIC_Software_JYSCL_PIN)            \
       : DL_GPIO_clearPins(IIC_Software_PORT, IIC_Software_JYSCL_PIN))
#define SDA(BIT)                                                               \
  (BIT ? DL_GPIO_setPins(IIC_Software_PORT, IIC_Software_JYSDA_PIN)            \
       : DL_GPIO_clearPins(IIC_Software_PORT, IIC_Software_JYSDA_PIN))
#define SDA_GET()                                                              \
  ((DL_GPIO_readPins(IIC_Software_PORT, IIC_Software_JYSDA_PIN) &              \
    IIC_Software_JYSDA_PIN)                                                    \
       ? 1                                                                     \
       : 0)

/* ==================== 姿态与角速度数据结构 (Data Structures) ==================== */
typedef struct {
  float x;       // 横滚角 Roll (单位: 度)
  float y;       // 俯仰角 Pitch (单位: 度)
  float z;       // 瞬时航向角 Yaw (单位: 度, 范围: -180 ~ +180)
  float total_z; // 全行程无缝累计航向角 (单位: 度, 已补偿过零突变)
  float gyro_z;  // Z 轴滤波后角速度 (单位: 度/秒)
} Gyro_Struct;

/* ==================== 外部变量声明 (External Declarations) ==================== */
extern Gyro_Struct *JY61P_Data;

/* ==================== 驱动层对外接口声明 (Function Prototypes) ==================== */
void jy61pInit(void);
uint8_t readDataJy61p(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
uint8_t writeDataJy61p(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
uint8_t jy61p_Init_Process(void);
Gyro_Struct *get_angle(void);

#endif // __BSP_GYRO_H__
