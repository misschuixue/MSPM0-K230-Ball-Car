/**
 * @file motor_ctrl.c
 * @brief 电机驱动与运动学闭环控制抽象层 (Motor Control & Kinematic Closed-Loop
 * BSP)
 * @note 该模块负责底层硬件 PWM 驱动、运动学解算以及所有相关的 PID 状态机控制。
 */

#include "motor_ctrl.h"
#include "main.h"
#include "smd.h"
#include <math.h>

// ==================== 外部系统状态与驱动层依赖 (External Dependencies)
// ====================
extern uint8_t Max_Speed_Test_Flag;              // 最大速度悬空测试标志
extern uint8_t K230_Ctrl_Mode;                   // K230 控制模式状态
extern volatile uint32_t ms_ticks;               // 系统全局滴答时钟
extern uint8_t Car_Mode;                         // 运动学闭环全局状态机模式
extern volatile float Motor1_Speed;              // 左轮真实转速
extern void BLE_send_String(unsigned char *str); // 蓝牙串口通信

// ==================== 模块内部私有状态 (Private States) ====================
static uint8_t motor1_enable_flag = 0;
static uint8_t motor2_enable_flag = 0;

int32_t PD42S1_Motor2_Pos = 22300; // 记录摆杆电机的固定目标角度

// ==================== 全局串级 PID 标志位 (Cascade PID Flags)
// ====================
uint8_t Turn_PID_Flag = 0;
uint8_t Distance_PID_Flag = 0;
uint8_t Gyro_PID_Flag = 0;
uint8_t Angle_PID_Flag = 0;
uint8_t Stand_PID_Flag = 0;

// ==================== PID 控制器实例定义 (PID Controller Instances)
// ====================

// 速度环 PID 控制器 (基于编码器脉冲)
pid_t pid_Motor1_Speed = {.Kp = 45.6f,
                          .Ki = 2.26f,
                          .Kd = 66.62f,
                          .Target = 0,
                          .Measure = 0,
                          .Error = 0,
                          .KpOut = 0,
                          .KiOut = 0,
                          .KdOut = 0,
                          .PID_Out = 0,
                          .PID_Limit_MAX = 9999,
                          .Ki_Limit_MAX = 9000};

pid_t pid_Motor2_Speed = {.Kp = 45.6f,
                          .Ki = 2.26f,
                          .Kd = 66.62f,
                          .Target = 0,
                          .Measure = 0,
                          .Error = 0,
                          .KpOut = 0,
                          .KiOut = 0,
                          .KdOut = 0,
                          .PID_Out = 0,
                          .PID_Limit_MAX = 9999,
                          .Ki_Limit_MAX = 9000};

// 驻车角度 PID 控制器 (基于陀螺仪维持绝对航向)
pid_t pid_Stand_Angle = {.Kp = 5.0f,
                         .Ki = 0.0f,
                         .Kd = 10.0f,
                         .Target = 0,
                         .Measure = 0,
                         .Error = 0,
                         .KpOut = 0,
                         .KiOut = 0,
                         .KdOut = 0,
                         .PID_Out = 0,
                         .PID_Limit_MAX = 400,
                         .Ki_Limit_MAX = 50};

// 寻线转向环 PID 控制器 (基于灰度传感器)
// 针对RWD后驱车修正：引入Ki彻底消除稳态转向不足，Kp保持35，Kd保持60
pid_t pid_Turn = {.Kp = 40.0f,
                  .Ki = 0.5f,
                  .Kd = 100.0f,
                  .Target = 0,
                  .Measure = 0,
                  .Error = 0,
                  .KpOut = 0,
                  .KiOut = 0,
                  .KdOut = 0,
                  .PID_Out = 0,
                  .PID_Limit_MAX = 1000,
                  .Ki_Limit_MAX = 1000};

// =========================================================================
// 【距离环 (定长位移) 控制器核心参数】 - 小车“自动驾驶”的驾驶舱
// =========================================================================
pid_t pid_Distance = {
    // 1. 【刹车时机与滑行距离】
    // Kp 越大，刹车越晚、越急；Kp 越小，刹车越早、滑行越温柔。
    // 推荐值：2.5f (丝滑)，如果改大请务必配合修改下方的限幅。
    .Kp = 2.5f,
    .Ki = 0.0f, // 串级系统中外环坚决不要加 I，避免积分饱和导致严重过冲
    .Kd = 0.0f, // 串级系统中外环坚决不要加 D，避免引入加速度引起震荡打滑

    .Target = 0,
    .Measure = 0,
    .Error = 0,
    .KpOut = 0,
    .KiOut = 0,
    .KdOut = 0,
    .PID_Out = 0,

    // 2. 【最高巡航极速限制 (Top Speed)】
    // 单位：mm/s。限制小车在长距离行驶时的最高速度。
    // 改为 100：龟速观摩模式；改为 500~1000：电赛狂飙模式。
    .PID_Limit_MAX = 200,
    .Ki_Limit_MAX = 30};

// 角速度外环 PID 控制器
float Gyro_Feedforward_Coefficient = 1.2f; // 前馈比例系数
pid_t pid_Gyro = {.Kp = 1.0f,
                  .Ki = 0.01f,
                  .Kd = 0.0f,
                  .Target = 0,
                  .Measure = 0,
                  .Error = 0,
                  .KpOut = 0,
                  .KiOut = 0,
                  .KdOut = 0,
                  .PID_Out = 0,
                  .PID_Limit_MAX = 1000,
                  .Ki_Limit_MAX = 300};

// 绝对角度锁定外环 PID 控制器
pid_t pid_Angle = {.Kp = 3.0f,
                   .Ki = 0.0f,
                   .Kd = 15.0f,
                   .Target = 0,
                   .Measure = 0,
                   .Error = 0,
                   .KpOut = 0,
                   .KiOut = 0,
                   .KdOut = 0,
                   .PID_Out = 0,
                   .PID_Limit_MAX = 3000,
                   .Ki_Limit_MAX = 3000};

// 球杆系统专用 PID 控制器 (已针对降低高频震荡优化)
pid_t pid_Ball_Beam = {.Kp = 0.6f,
                       .Ki = 0.0f,
                       .Kd = 65.0f,
                       .Target = 320,
                       .Measure = 0,
                       .Error = 0,
                       .KpOut = 0,
                       .KiOut = 0,
                       .KdOut = 0,
                       .PID_Out = 0,
                       .PID_Limit_MAX = 200,
                       .Ki_Limit_MAX = 50};

// ==================== 电机使能与失能底层控制 (Motor Enable/Disable)
// ====================
void Motors_Enable(void) {
  motor1_enable_flag = 1;
  motor2_enable_flag = 1;
}
void Motors_Disable(void) {
  motor1_enable_flag = 0;
  motor2_enable_flag = 0;
}
void Motor1_Enable(void) { motor1_enable_flag = 1; }
void Motor2_Enable(void) { motor2_enable_flag = 1; }
void Motor1_Disable(void) { motor1_enable_flag = 0; }
void Motor2_Disable(void) { motor2_enable_flag = 0; }

// ==================== 统一云台与底盘速度分发接口 (Unified Speed Dispatcher)
// ====================
/**
 * @brief  K230 云台专用的统一速度下发接口
 * @param  speed_L: X轴 Yaw 偏航目标速度
 * @param  speed_R: Y轴 Pitch 俯仰目标速度
 */
void Car_Set_Speed_Unified(float speed_L, float speed_R) {
  // 拦截卫哨：上电初始化位置模式(0)时严禁后台强刷速度，但任务三必须放行
  if (K230_Ctrl_Mode == 0 && Car_Mode != Ball_Task2_Mode)
    return;

  // 处理 X 轴电机 (UART1)
  g_smd_target_uart = 1;
  uint8_t dir_L = (speed_L >= 0) ? 0 : 1;
  smd_speed_mode(1, dir_L, 5, (uint32_t)fabs(speed_L));

  // ==================== 摆杆电机绝对位置控制与限位保护 ====================
  extern int32_t PD42S1_Motor2_Pos;
  int32_t target_pos = PD42S1_Motor2_Pos;

  if (Car_Mode == Ball_Task2_Mode) {
    // 【关键修复】任务三：球杆系统属于双积分模型。PID的输出必须直接映射为"摆杆的物理倾角"(绝对位置)。
    // 绝不能对其进行积分(那会变成 PI
    // 控制器，导致理论上无休止的等速震荡往返运动)。 speed_R 就是 PID
    // 计算出来的角度需求。25.0f 是"PID输出 -> 脉冲数"的机械放大系数。
    // 增大此系数可减小纯P控制的稳态静差(摆杆倾斜更多→推力更大→克服静摩擦)。
    target_pos = PD42S1_Motor2_Pos + (int32_t)(speed_R * 25.0f);
  } else {
    // 遥控/视觉速度模式：speed_R 是期望速度，需要转化为脉冲增量积分到基准位置中
    int32_t step_increment = (int32_t)(speed_R * 0.8f);
    PD42S1_Motor2_Pos += step_increment;
    target_pos = PD42S1_Motor2_Pos;
  }

  // 以 PD42S1_Motor2_Pos 为中心基准，设定安全限位（暂定 ±3500 脉冲，杜绝撞杆）
  int32_t base_pos = PD42S1_Motor2_Pos;
  if (target_pos > base_pos + 3500)
    target_pos = base_pos + 3500;
  if (target_pos < base_pos - 3500)
    target_pos = base_pos - 3500;

  // 对于积分模式，把限位后的值写回，以防积分饱和越界
  if (Car_Mode != Ball_Task2_Mode) {
    PD42S1_Motor2_Pos = target_pos;
  }

  // 处理 Y 轴电机 (UART2)，下发绝对位置模式指令
  g_smd_target_uart = 2;
  uint16_t motor2_speed = 300; // 任务三闭环需要极高响应，固定 300 RPM 跟手
  if (Car_Mode != Ball_Task2_Mode) {
    motor2_speed = (uint16_t)fabs(speed_R);
    if (motor2_speed < 80)
      motor2_speed = 80;
    if (motor2_speed > 250)
      motor2_speed = 250;
  }

  // 任务三保活: 每500ms(50次)插入一次清状态指令, 防止电机驱动器
  // 累积内部错误(位置误差等)后自动失能。该tick不发pos_mode, 避免帧竞争。
  static uint16_t task3_keepalive_cnt = 0;
  if (Car_Mode == Ball_Task2_Mode) {
    if (++task3_keepalive_cnt >= 50) {
      task3_keepalive_cnt = 0;
      smd_clear_sta(1);           // 清除堵转/失能/刹车状态
      smd_remove_clog_protect(1); // 再次解除堵转保护
    } else {
      smd_pos_mode(1, 0, 5, motor2_speed, target_pos);
    }
  } else {
    smd_pos_mode(1, 0, 5, motor2_speed, target_pos);
    task3_keepalive_cnt = 0;
  }
}

// ==================== PID 核心运算逻辑 (PID Core Logic) ====================

/**
 * @brief 通用浮点数限幅保护
 */
void PID_Limit(float *a, float ABS_MAX) {
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < -ABS_MAX)
    *a = -ABS_MAX;
}

/**
 * @brief 位置式 PID 计算核心算子
 * @param pid: 控制器实例指针
 * @param Measure: 实际测量值反馈
 * @param Target: 目标期望值 Setpoint
 * @retval 总体控制输出量 PID_Out
 */
float PID_Calculate(pid_t *pid, float Measure, float Target) {
  pid->Target = Target;
  float dMeasure = Measure - pid->Measure; // 取一阶微分消除跃变冲击
  pid->Measure = Measure;
  pid->Error = Target - Measure;

  // 比例项运算
  pid->KpOut = pid->Kp * pid->Error;

  // 积分项运算
  // 【致命漏洞修复】：原代码当 Target == 0
  // 时强制清空积分，导致循迹环(目标永远为0)的积分项永远失效！
  // 现已移除该限制，允许任何有效目标值的正常积分累加。直道防震荡的“积分死区”已专门在
  // timer.c 中实现。
  pid->KiOut += pid->Ki * pid->Error;
  PID_Limit(&(pid->KiOut), (float)pid->Ki_Limit_MAX);

  // 微分项运算 (基于测量值微分，避免 Target 阶跃引起超调)
  pid->KdOut = -(pid->Kd * dMeasure);

  // 汇总与限幅
  pid->PID_Out = pid->KpOut + pid->KiOut + pid->KdOut;
  PID_Limit(&(pid->PID_Out), (float)pid->PID_Limit_MAX);

  return pid->PID_Out;
}

/**
 * @brief 动态重置 PID 参数 (常用于上位机调参)
 */
void Set_PID_Param(pid_t *pid, float P, float I, float D) {
  pid->Kp = P;
  pid->Ki = I;
  pid->Kd = D;
  pid->KpOut = 0.0f;
  pid->KiOut = 0.0f;
  pid->KdOut = 0.0f;
  pid->PID_Out = 0.0f;
}

/**
 * @brief 强制清零 PID 历史状态 (状态机切换时调用)
 */
void PID_Reset(pid_t *pid) {
  pid->KpOut = 0.0f;
  pid->KiOut = 0.0f;
  pid->KdOut = 0.0f;
  pid->PID_Out = 0.0f;
  pid->Error = 0.0f;
  pid->Measure = 0.0f; // 清零测量历史，防止微分项起步尖峰
}

// ==================== 电机底层 PWM 操作 (Hardware PWM Layer)
// ====================

/**
 * @brief 通用整数限幅保护
 */
void PWM_Limit(int *a, int ABS_MAX) {
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < -ABS_MAX)
    *a = -ABS_MAX;
}

void Set_Motor1_PWM(int Target_PWM) {
  PWM_Limit(&Target_PWM, 9999);
  DL_TimerA_setCaptureCompareValue(PWM_0_INST, Target_PWM, GPIO_PWM_0_C1_IDX);
}

void Set_Motor2_PWM(int Target_PWM) {
  PWM_Limit(&Target_PWM, 9999);
  DL_TimerA_setCaptureCompareValue(PWM_0_INST, Target_PWM, GPIO_PWM_0_C0_IDX);
}

void Set_Motor1_Speed(int Target_Speed) {
  if (motor1_enable_flag == 1 || Max_Speed_Test_Flag == 1) {
    if (Target_Speed >= 0) {
      Set_Motor1_PWM(Target_Speed);
      Motor1_Forward();
    } else {
      Set_Motor1_PWM(-Target_Speed);
      Motor1_Backward();
    }
  } else {
    Set_Motor1_PWM(0);
    Motor1_Stop();
    pid_Motor1_Speed.KpOut = 0.0f;
    pid_Motor1_Speed.KiOut = 0.0f;
    pid_Motor1_Speed.KdOut = 0.0f;
    pid_Motor1_Speed.PID_Out = 0.0f;
    pid_Motor1_Speed.Error = 0.0f;
  }
}

void Set_Motor2_Speed(int Target_Speed) {
  if (motor2_enable_flag == 1 || Max_Speed_Test_Flag == 1) {
    if (Target_Speed >= 0) {
      Set_Motor2_PWM(Target_Speed);
      Motor2_Forward();
    } else {
      Set_Motor2_PWM(-Target_Speed);
      Motor2_Backward();
    }
  } else {
    Set_Motor2_PWM(0);
    Motor2_Stop();
    pid_Motor2_Speed.KpOut = 0.0f;
    pid_Motor2_Speed.KiOut = 0.0f;
    pid_Motor2_Speed.KdOut = 0.0f;
    pid_Motor2_Speed.PID_Out = 0.0f;
    pid_Motor2_Speed.Error = 0.0f;
  }
}

void SET_MOTORS_SPEED(int Target_Motor1_Speed, int Target_Motor2_Speed) {
  Set_Motor1_Speed(Target_Motor1_Speed);
  Set_Motor2_Speed(Target_Motor2_Speed);
}

// ==================== ZN法自动整定器 (Ziegler-Nichols Auto-Tuner)
// ====================
#include <stdio.h>
#define AT_TARGET_SPEED 400.0f
#define AT_PWM_AMPLITUDE 3500.0f
#define AT_SAMPLE_TIME 0.01f

typedef enum {
  AT_STATE_IDLE = 0,
  AT_STATE_WAIT_CROSS,
  AT_STATE_MEASURE,
  AT_STATE_DONE
} AutoTuneState;

static AutoTuneState at_state = AT_STATE_IDLE;
static float max_speed = 0.0f;
static float min_speed = 0.0f;
static uint32_t cross_count = 0;
static uint32_t first_cross_time = 0;
static uint32_t last_cross_time = 0;
static float last_speed = 0.0f;
static int relay_state = 1;

void AutoTune_Init(void) {
  at_state = AT_STATE_WAIT_CROSS;
  max_speed = 0.0f;
  min_speed = 9999.0f;
  cross_count = 0;
  last_speed = Motor1_Speed;
  relay_state = (Motor1_Speed < AT_TARGET_SPEED) ? 1 : -1;
  BLE_send_String((unsigned char *)"[AutoTune] Started...\r\n");
}

void AutoTune_Process(float *pwm1, float *pwm2) {
  if (at_state == AT_STATE_IDLE || at_state == AT_STATE_DONE) {
    *pwm1 = 0.0f;
    *pwm2 = 0.0f;
    return;
  }
  float current_speed = Motor1_Speed;
  if ((last_speed < AT_TARGET_SPEED && current_speed >= AT_TARGET_SPEED) ||
      (last_speed >= AT_TARGET_SPEED && current_speed < AT_TARGET_SPEED)) {
    if (at_state == AT_STATE_WAIT_CROSS) {
      at_state = AT_STATE_MEASURE;
      first_cross_time = ms_ticks;
      cross_count = 1;
      max_speed = current_speed;
      min_speed = current_speed;
    } else if (at_state == AT_STATE_MEASURE) {
      cross_count++;
      last_cross_time = ms_ticks;
      if (cross_count >= 9) {
        at_state = AT_STATE_DONE;
        float Tu = (float)(last_cross_time - first_cross_time) / 1000.0f / 4.0f;
        float A = (max_speed - min_speed) / 2.0f;
        if (A < 0.1f)
          A = 0.1f;
        float Ku = (4.0f * AT_PWM_AMPLITUDE) / (3.1415926f * A);

        // Tyreus-Luyben 整定公式 (无超调优化)
        float Kp_TL = Ku / 3.2f;
        float Ti_TL = 2.2f * Tu;
        float Td_TL = Tu / 6.3f;
        float Ki_TL = Kp_TL * (AT_SAMPLE_TIME / Ti_TL);
        float Kd_TL = Kp_TL * (Td_TL / AT_SAMPLE_TIME);

        if (Kp_TL > 999.0f)
          Kp_TL = 999.0f;
        if (Ki_TL > 999.0f)
          Ki_TL = 999.0f;
        if (Kd_TL > 999.0f)
          Kd_TL = 999.0f;
        if (Kp_TL < 0.0f)
          Kp_TL = 0.0f;
        if (Ki_TL < 0.0f)
          Ki_TL = 0.0f;
        if (Kd_TL < 0.0f)
          Kd_TL = 0.0f;

        Set_PID_Param(&pid_Motor1_Speed, Kp_TL, Ki_TL, Kd_TL);
        Set_PID_Param(&pid_Motor2_Speed, Kp_TL, Ki_TL, Kd_TL);

        // 整定完毕自动切回速度环 (Speed_Mode)
        Car_Mode = 1;
        *pwm1 = 0.0f;
        *pwm2 = 0.0f;
        return;
      }
    }
    relay_state = -relay_state;
  }
  if (at_state == AT_STATE_MEASURE) {
    if (current_speed > max_speed)
      max_speed = current_speed;
    if (current_speed < min_speed)
      min_speed = current_speed;
  }
  last_speed = current_speed;
  float out_pwm = (relay_state > 0) ? AT_PWM_AMPLITUDE : -AT_PWM_AMPLITUDE;
  *pwm1 = out_pwm;
  *pwm2 = out_pwm;
}