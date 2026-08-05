/**
 * @file bsp_gyro.c
 * @brief 六轴陀螺仪姿态感知驱动实现 (JY61P IMU BSP)
 * @note 负责软件模拟 I2C 时序驱动、JY61P 非阻塞初始化状态机、姿态角解算及一阶滤波。
 */

#include "bsp_gyro.h"
#include "main.h"
#include "stdio.h"
#include "string.h"

// ==================== 模块内部私有状态与外部依赖 (Private States) ====================
extern volatile uint32_t ms_ticks; // 引用系统滴答定时器

static Gyro_Struct Gyro_Structure; // 姿态解算全局数据存储结构体

// ==================== 非阻塞初始化状态机 (FSM Initialization) ====================

/**
 * @brief  JY61P 传感器非阻塞硬件初始化状态机
 * @note   利用系统滴答时钟分步执行传感器配置指令，避免开机延时阻塞主循环
 * @return 0: 初始化进行中; 1: 初始化全部完成
 */
uint8_t jy61p_Init_Process(void) {
  static uint8_t init_state = 0;
  static uint32_t wait_start_time = 0;
  uint8_t unlock_reg[2] = {0x88, 0xB5};
  uint8_t z_axis_reg[2] = {0x04, 0x00};
  uint8_t angle_reg[2] = {0x08, 0x00};
  uint8_t save_reg[2] = {0x00, 0x00};

  switch (init_state) {
  case 0: // 状态 0: 发送 Z 轴解锁指令
    writeDataJy61p(IIC_ADDR, UN_REG, unlock_reg, 2);
    wait_start_time = ms_ticks;
    init_state = 1;
    break;

  case 1: // 状态 1: 等待 200ms 后发送 Z 轴归零指令
    if (ms_ticks - wait_start_time >= 200) {
      writeDataJy61p(IIC_ADDR, ANGLE_REFER_REG, z_axis_reg, 2);
      wait_start_time = ms_ticks;
      init_state = 2;
    }
    break;

  case 2: // 状态 2: 等待 200ms 后发送 Z 轴配置保存指令
    if (ms_ticks - wait_start_time >= 200) {
      writeDataJy61p(IIC_ADDR, SAVE_REG, save_reg, 2);
      wait_start_time = ms_ticks;
      init_state = 3;
    }
    break;

  case 3: // 状态 3: 等待 200ms 后发送角度解锁指令
    if (ms_ticks - wait_start_time >= 200) {
      writeDataJy61p(IIC_ADDR, UN_REG, unlock_reg, 2);
      wait_start_time = ms_ticks;
      init_state = 4;
    }
    break;

  case 4: // 状态 4: 等待 200ms 后发送角度归零指令
    if (ms_ticks - wait_start_time >= 200) {
      writeDataJy61p(IIC_ADDR, ANGLE_REFER_REG, angle_reg, 2);
      wait_start_time = ms_ticks;
      init_state = 5;
    }
    break;

  case 5: // 状态 5: 等待 200ms 后发送角度配置保存指令
    if (ms_ticks - wait_start_time >= 200) {
      writeDataJy61p(IIC_ADDR, SAVE_REG, save_reg, 2);
      wait_start_time = ms_ticks;
      init_state = 6;
    }
    break;

  case 6: // 状态 6: 保持 200ms 稳定窗口，清空数据结构体并完成初始化
    if (ms_ticks - wait_start_time >= 200) {
      memset((void *)&Gyro_Structure, 0, sizeof(Gyro_Structure));
      init_state = 0; // 重置状态机以便下次调用
      return 1;       // 返回 1 表示初始化全部完成
    }
    break;
  }
  return 0; // 返回 0 表示初始化仍在进行中
}

/**
 * @brief  JY61P 阻塞式初始化包装接口
 * @note   循环推进状态机直至完成，兼容上电初始化调用
 */
void jy61pInit(void) {
  while (jy61p_Init_Process() == 0)
    ;
}

// ==================== 软件模拟 I2C 协议驱动层 (Software I2C Bit-Banging) ====================

/**
 * @brief  产生 I2C 总线起始信号
 */
void IIC_Start(void) {
  SDA_OUT();

  SCL(0);
  SDA(1);
  SCL(1);

  delay_us(5);

  SDA(0);
  delay_us(5);
  SCL(0);
  delay_us(5);
}

/**
 * @brief  产生 I2C 总线停止信号
 */
void IIC_Stop(void) {
  SDA_OUT();
  SCL(0);
  SDA(0);

  SCL(1);
  delay_us(5);
  SDA(1);
  delay_us(5);
}

/**
 * @brief  主机发送应答信号 (ACK / NACK)
 * @param  ack: 0 发送 ACK; 1 发送 NACK
 */
void IIC_Send_Ack(unsigned char ack) {
  SDA_OUT();
  SCL(0);
  SDA(0);
  delay_us(5);
  if (!ack)
    SDA(0);
  else
    SDA(1);
  SCL(1);
  delay_us(5);
  SCL(0);
  SDA(1);
}

/**
 * @brief  等待从机应答信号
 * @return 0: 收到有效应答; 1: 等待应答超时
 */
unsigned char I2C_WaitAck(void) {
  char ack = 0;
  char ack_flag = 50;

  SDA_IN();

  SDA(1);
  while ((SDA_GET() == 1) && (ack_flag)) {
    ack_flag--;
    delay_us(5);
  }

  if (ack_flag == 0) {
    IIC_Stop();
    return 1;
  } else {
    SCL(1);
    delay_us(5);
    SCL(0);
    SDA_OUT();
  }
  return ack;
}

/**
 * @brief  I2C 写入单字节数据
 * @param  dat: 待发送的字节数据
 */
void Send_Byte(uint8_t dat) {
  int i = 0;
  SDA_OUT();
  SCL(0); // 拉低时钟线准备数据传输

  for (i = 0; i < 8; i++) {
    SDA((dat & 0x80) >> 7);
    delay_us(2);

    SCL(1);
    delay_us(5);

    SCL(0);
    delay_us(5);

    dat <<= 1;
  }
}

/**
 * @brief  I2C 读取单字节数据
 * @return 读取到的单字节数据
 */
unsigned char Read_Byte(void) {
  unsigned char i, receive = 0;
  SDA_IN(); // 配置 SDA 为输入模式
  for (i = 0; i < 8; i++) {
    SCL(0);
    delay_us(5);
    SCL(1);
    delay_us(5);
    receive <<= 1;
    if (SDA_GET()) {
      receive |= 1;
    }
    delay_us(5);
  }

  return receive;
}

// ==================== 传感器寄存器底层传输 (Register Transmission) ====================

/**
 * @brief  向 JY61P 指定寄存器连续写入数据
 * @param  dev: 从机设备 I2C 地址
 * @param  reg: 目标寄存器地址
 * @param  data: 待写入数据缓冲区首地址
 * @param  length: 写入数据字节数
 * @return 0: 写入失败; 1: 写入成功
 */
uint8_t writeDataJy61p(uint8_t dev, uint8_t reg, uint8_t *data,
                       uint32_t length) {
  uint32_t count = 0;

  IIC_Start();

  Send_Byte(dev << 1);
  if (I2C_WaitAck() == 1)
    return 0;

  Send_Byte(reg);
  if (I2C_WaitAck() == 1)
    return 0;

  for (count = 0; count < length; count++) {
    Send_Byte(data[count]);
    if (I2C_WaitAck() == 1)
      return 0;
  }

  IIC_Stop();

  return 1;
}

/**
 * @brief  从 JY61P 指定寄存器连续读取数据
 * @param  dev: 从机设备 I2C 地址
 * @param  reg: 目标寄存器地址
 * @param  data: 接收数据存储缓冲区首地址
 * @param  length: 读取数据字节数
 * @return 0: 读取失败; 1: 读取成功
 */
uint8_t readDataJy61p(uint8_t dev, uint8_t reg, uint8_t *data,
                       uint32_t length) {
  uint32_t count = 0;

  IIC_Start();

  Send_Byte((dev << 1) | 0);
  if (I2C_WaitAck() == 1)
    return 0;

  Send_Byte(reg);
  if (I2C_WaitAck() == 1)
    return 0;

  delay_us(5);

  IIC_Start();

  Send_Byte((dev << 1) | 1);
  if (I2C_WaitAck() == 1)
    return 0;

  for (count = 0; count < length; count++) {
    if (count != length - 1) {
      data[count] = Read_Byte();
      IIC_Send_Ack(0);
    } else {
      data[count] = Read_Byte();
      IIC_Send_Ack(1);
    }
  }

  IIC_Stop();

  return 1;
}

// ==================== 姿态角与角速度核心解算接口 (Kinematic Estimation) ====================

/**
 * @brief  读取并解算六轴姿态角及角速度
 * @note   包含上电稳态丢弃、±180度过零连续解包及角速度一阶低通滤波
 * @return 姿态结构体指针; 数据未就绪或读取失败返回 NULL
 */
Gyro_Struct *get_angle(void) {
  uint8_t sda_angle[6] = {0};
  int ret = 0;

  // 上电稳态丢弃缓冲区: 丢弃上电前 20 次读取周期，避开传感器未收敛瞬态噪声
  static uint8_t stable_count = 0;
  if (stable_count < 20) {
    stable_count++;
    return NULL;
  }

  // 清空数据接收缓存
  memset((void *)sda_angle, 0, sizeof(sda_angle));

  // 读取角度寄存器数据 (0x3D 开始连续 6 字节)
  ret = readDataJy61p(IIC_ADDR, 0x3D, sda_angle, 6);
  if (ret == 0) {
    return NULL;
  }

#if GYRO_DEBUG
  lc_printf("RollL = %x\r\n", sda_angle[0]);
  lc_printf("RollH = %x\r\n", sda_angle[1]);
  lc_printf("PitchL = %x\r\n", sda_angle[2]);
  lc_printf("PitchH = %x\r\n", sda_angle[3]);
  lc_printf("YawL = %x\r\n", sda_angle[4]);
  lc_printf("YawH = %x\r\n", sda_angle[5]);
#endif

  // 解算三轴姿态角并归一化至 [-180.0, +180.0] 区间
  float RollX = (float)(((sda_angle[1] << 8) | sda_angle[0]) / 32768.0 * 180.0);
  if (RollX > 180.0) {
    RollX -= 360.0;
  } else if (RollX < -180.0) {
    RollX += 360.0;
  }

  float PitchY =
      (float)(((sda_angle[3] << 8) | sda_angle[2]) / 32768.0 * 180.0);
  if (PitchY > 180.0) {
    PitchY -= 360.0;
  } else if (PitchY < -180.0) {
    PitchY += 360.0;
  }

  float YawZ = (float)(((sda_angle[5] << 8) | sda_angle[4]) / 32768.0 * 180.0);
  if (YawZ > 180.0) {
    YawZ -= 360.0;
  } else if (YawZ < -180.0) {
    YawZ += 360.0;
  }

  // 连续航向角无缝解包积分: 补偿 ±180 度跨越突变，计算全行程累计偏航角
  static float last_yaw_z = 0.0f;
  static uint8_t first_read = 1;
  if (first_read) {
    last_yaw_z = YawZ;
    Gyro_Structure.total_z = 0.0f;
    first_read = 0;
  } else {
    float delta = YawZ - last_yaw_z;
    // 检测并补偿 ±180 度过零突变
    if (delta > 180.0f)
      delta -= 360.0f;
    else if (delta < -180.0f)
      delta += 360.0f;

    Gyro_Structure.total_z += delta; // 累计真实偏航角度
    last_yaw_z = YawZ;               // 更新历史位姿参考值
  }

  // 读取硬件原生角速度寄存器 (0x37 开始连续 6 字节)
  uint8_t sda_gyro[6] = {0};
  int ret_gyro = readDataJy61p(IIC_ADDR, 0x37, sda_gyro, 6);
  if (ret_gyro != 0) {
    // 转换为角速度物理量 (度/秒, 量程: ±2000 dps)
    float GyroZ_HW =
        (float)((short)((sda_gyro[5] << 8) | sda_gyro[4])) / 32768.0f * 2000.0f;

    // 一阶低通滤波: 抑制车模电机运转时的高频机械震动
    static float filtered_gyro_hw = 0.0f;
    static uint8_t first_gyro_read = 1;
    if (first_gyro_read) {
      filtered_gyro_hw = GyroZ_HW;
      first_gyro_read = 0;
    } else {
      filtered_gyro_hw = filtered_gyro_hw * 0.85f + GyroZ_HW * 0.15f;
    }

    Gyro_Structure.gyro_z = filtered_gyro_hw;
  }

  // 更新姿态解算全局数据
  Gyro_Structure.x = RollX;
  Gyro_Structure.y = PitchY;
  Gyro_Structure.z = YawZ;

  return &Gyro_Structure;
}
