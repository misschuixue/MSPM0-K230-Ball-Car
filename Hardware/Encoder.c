#include "Encoder.h"
#include "timer.h"

volatile uint32_t motor1_latest_pulse_micros = 0;
volatile uint32_t motor2_latest_pulse_micros = 0;
volatile int32_t Motor1_Encoder_Value = 0;
volatile int32_t Motor2_Encoder_Value = 0;

volatile float Motor1_Speed = 0;
volatile float Motor2_Speed = 0;
volatile float Motor1_Lucheng = 0;
volatile float Motor2_Lucheng = 0;
volatile float Measure_Distance = 0;

// 外部中断服务函数 (处理编码器脉冲计数)
void GROUP1_IRQHandler(void) {

  if (DL_Interrupt_getStatusGroup(DL_INTERRUPT_GROUP_1,
                                  DL_INTERRUPT_GROUP1_GPIOA)) {
    uint32_t Encoder_GPIO_Int =
        DL_GPIO_getEnabledInterruptStatus(GPIOA, 0xFFFFFFFF);
    // 通道1: 右轮 (Motor2) A相
    if ((Encoder_GPIO_Int & Encoder_A_PIN) == Encoder_A_PIN) {
      DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_A_PIN);
      if (Read_Encoder_A == 1) { // 上升沿
        if (Read_Encoder_B == 0) {
          Motor2_Encoder_Value++;
        } else if (Read_Encoder_B == 1) {
          Motor2_Encoder_Value--;
        }
      } else if (Read_Encoder_A == 0) { // 下降沿
        if (Read_Encoder_B == 0) {
          Motor2_Encoder_Value--;
        } else if (Read_Encoder_B == 1) {
          Motor2_Encoder_Value++;
        }
      }
      motor2_latest_pulse_micros = get_micros();
    }

    // 通道2: 右轮 (Motor2) B相
    if ((Encoder_GPIO_Int & Encoder_B_PIN) == Encoder_B_PIN) {
      DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_B_PIN);
      if (Read_Encoder_B == 1) { // 上升沿
        if (Read_Encoder_A == 0) {
          Motor2_Encoder_Value--;
        } else if (Read_Encoder_A == 1) {
          Motor2_Encoder_Value++;
        }
      } else if (Read_Encoder_B == 0) { // 下降沿
        if (Read_Encoder_A == 0) {
          Motor2_Encoder_Value++;
        } else if (Read_Encoder_A == 1) {
          Motor2_Encoder_Value--;
        }
      }
      motor2_latest_pulse_micros = get_micros();
    }

    // 通道3: 左轮 (Motor1) A相
    if ((Encoder_GPIO_Int & Encoder_C_PIN) == Encoder_C_PIN) {
      DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_C_PIN);
      if (Read_Encoder_C == 1) { // 上升沿
        if (Read_Encoder_D == 0) {
          Motor1_Encoder_Value++;
        } else if (Read_Encoder_D == 1) {
          Motor1_Encoder_Value--;
        }
      } else if (Read_Encoder_C == 0) { // 下降沿
        if (Read_Encoder_D == 0) {
          Motor1_Encoder_Value--;
        } else if (Read_Encoder_D == 1) {
          Motor1_Encoder_Value++;
        }
      }
      motor1_latest_pulse_micros = get_micros();
    }

    // 通道4: 左轮 (Motor1) B相
    if ((Encoder_GPIO_Int & Encoder_D_PIN) == Encoder_D_PIN) {
      DL_GPIO_clearInterruptStatus(Encoder_PORT, Encoder_D_PIN);
      if (Read_Encoder_D == 1) { // 上升沿
        if (Read_Encoder_C == 0) {
          Motor1_Encoder_Value--;
        } else if (Read_Encoder_C == 1) {
          Motor1_Encoder_Value++;
        }
      } else if (Read_Encoder_D == 0) { // 下降沿
        if (Read_Encoder_C == 0) {
          Motor1_Encoder_Value++;
        } else if (Read_Encoder_C == 1) {
          Motor1_Encoder_Value--;
        }
      }
      motor1_latest_pulse_micros = get_micros();
    }
  }
}

volatile long long Motor1_Total_Pulse = 0;
volatile long long Motor2_Total_Pulse = 0;

// 计算左轮当前速度 (M法测速 + 一阶低通滤波)
void Motor1_Get_Speed(void) {
  int32_t Encoder_TIM;
  NVIC_DisableIRQ(GPIOA_INT_IRQn);
  Encoder_TIM = Motor1_Encoder_Value;
  Motor1_Encoder_Value = 0;
  NVIC_EnableIRQ(GPIOA_INT_IRQn);
  Motor1_Total_Pulse += Encoder_TIM;

  // M法测速公式：delta / SAMPLE_TIME = counts/s
  float speed_count = (float)Encoder_TIM / SAMPLE_TIME;
  float raw_speed = speed_count / COUNTS_PER_MM; // 转化为 mm/s

  // 一阶低通滤波
  Motor1_Speed = (0.1f * raw_speed) + (0.9f * Motor1_Speed);
}

// 计算右轮当前速度 (M法测速 + 一阶低通滤波)
void Motor2_Get_Speed(void) {
  int32_t Encoder_TIM;
  NVIC_DisableIRQ(GPIOA_INT_IRQn);
  Encoder_TIM = Motor2_Encoder_Value;
  Motor2_Encoder_Value = 0;
  NVIC_EnableIRQ(GPIOA_INT_IRQn);
  Motor2_Total_Pulse += Encoder_TIM;

  // M法测速公式：delta / SAMPLE_TIME = counts/s
  float speed_count = (float)Encoder_TIM / SAMPLE_TIME;
  float raw_speed = -(speed_count / COUNTS_PER_MM); // 右轮反向

  // 一阶低通滤波
  Motor2_Speed = (0.1f * raw_speed) + (0.9f * Motor2_Speed);
}

// 计算当前小车行驶的里程
void MEASURE_MOTORS_SPEED(void) {
  Motor1_Get_Speed();
  Motor2_Get_Speed();

  Motor1_Lucheng += Motor1_Speed * SAMPLE_TIME; // 路程累计
  Motor2_Lucheng += Motor2_Speed * SAMPLE_TIME; // 路程累计

  /* 获取底层的原始累计里程 */
  float raw_distance = Motor1_Lucheng / 2.0f + Motor2_Lucheng / 2.0f;

  /* =========================================================================
   * 双向独立里程计校准 (Bidirectional Odometry Calibration)
   * 补偿小车在前进与后退时因重心或轮胎纹理导致的非对称滑移
   * =========================================================================
   */
  static float last_raw_distance = 0.0f;
  float delta_raw = raw_distance - last_raw_distance;
  last_raw_distance = raw_distance;

  /* 物理标定系数 (实测物理距离 / 编码器理论距离) */
  float Forward_Ratio = 195.0f / 200.0f;
  float Backward_Ratio = 195.0f / 200.0f;

  if (delta_raw > 0) {
    Measure_Distance += delta_raw * Forward_Ratio;
  } else {
    Measure_Distance += delta_raw * Backward_Ratio;
  }

  /* 防止累计误差过大导致的溢出，定时清零 */
  if (Measure_Distance > 10000.0f) {
    Measure_Distance = 0;
    Motor1_Lucheng = 0;
    Motor2_Lucheng = 0;
    last_raw_distance = 0.0f; /* 同步清零静态基准点，防止产生巨大 delta 跳变 */
  }
}
