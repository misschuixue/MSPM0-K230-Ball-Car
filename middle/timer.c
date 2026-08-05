/**
 * @file timer.c
 * @brief 定时器中断及核心控制任务调度
 * @details
 * 处理系统定时任务，包括按键扫描、传感器读取、PID控制计算以及电机输出控制等。
 * @author WILLiam
 * @date 2026-06-12
 */

#include "timer.h"
#include "Encoder.h"
#include "bsp_gyro.h"
#include "gw_gray.h"
#include "key.h"
#include "math.h"
#include "motor_ctrl.h"
#include "protocol.h"
#include "task.h"
#include "usart.h"

// ==================== 全局变量 ====================

uint16_t count_10ms = 0;
uint16_t count_100ms = 0;
uint16_t t = 0;
uint16_t Speed_Mode_Run_Timer = 0; // 速度环模式下的测速运行计时器
volatile uint32_t ms_ticks = 0;
volatile int32_t K230_Target_Pos_X = 0;
volatile int32_t K230_Target_Pos_Y = 0;
volatile uint8_t K230_New_Pos_Flag = 0;
uint8_t Max_Speed_Test_Flag = 0; // 0: 关闭最大速度悬空测试，恢复正常 PID 控制
float target_gyro_ramp = 0.0f;
float soft_target_gyro = 0.0f;

uint32_t get_micros(void) {
  uint32_t ms = ms_ticks;
  uint32_t count = DL_TimerG_getTimerCount(TIMER_0_INST);

  // Use getRawInterruptStatus to check without clearing the hardware flag!
  if (DL_TimerG_getRawInterruptStatus(TIMER_0_INST,
                                      DL_TIMERG_INTERRUPT_ZERO_EVENT)) {
    count = DL_TimerG_getTimerCount(TIMER_0_INST);
    ms++;
  } else if (ms != ms_ticks) {
    ms = ms_ticks;
    count = DL_TimerG_getTimerCount(TIMER_0_INST);
  }

  return (ms * 1000) + (count * 1000) / 625;
}

volatile float Target_ChaSu;        // 目标差速
volatile float Motor1_Target_Speed; // 左边电机目标速度
volatile float Motor2_Target_Speed; // 右边电机目标速度

uint8_t huidu_state = 0;
volatile uint32_t control_cnt = 0;

// 全局运行标志位（供主循环轮询使用）
volatile uint8_t flag_5ms_gyro_read = 0;
volatile uint8_t flag_1s_printf = 0;
volatile uint8_t flag_50ms_telemetry = 0;
volatile uint8_t flag_50ms_lcd = 0;
volatile uint16_t telemetry_pause_ms = 0;

float Task1_Time_Sec = 0.0f; // 任务一用时
float Task2_Time_Sec = 0.0f; // 任务二用时
float Task1_Dist = 0.0f;     // 任务一当前路程
float Task2_Dist = 0.0f;     // 任务二当前路程

// ==================== 外部变量声明 ====================
extern float Basic_Speed;        // 电机目标速度
extern uint8_t OLED_View_Select; // OLED选择界面变量
extern float Target_Distance;
extern float Target_Gyro;
extern float Target_Angle;

extern uint8_t Turn_PID_Flag;
extern uint8_t Angle_PID_Flag;
extern pid_t pid_Turn;
extern pid_t pid_Distance;
extern pid_t pid_Angle;
extern Gyro_Struct *JY61P_Data; // 陀螺仪数据结构体

// 蓝牙BT指令标志: 用户手动设置过Target后置1, 取消默认覆盖
uint8_t bt_user_override = 0;

extern volatile bool g_rx_frame_flag;
extern int16_t Gimbal_Speed_X; // 云台控制目标速度变量X
extern int16_t Gimbal_Speed_Y; // 云台控制目标速度变量Y
extern volatile uint8_t ble_rx_idle_cnt;
extern uint8_t Test_Speed_Mode;
extern float Target_Speed_Test;
extern void delay_cycles(uint32_t);
extern uint8_t Tracking_Test_Flag;
extern uint8_t Tuning_State;
extern void BLE_send_String(unsigned char *str);

// 从其他文件引入的变量
extern float Gyro_Feedforward_Coefficient;
extern volatile long long Motor1_Total_Pulse;
extern volatile long long Motor2_Total_Pulse;

// ==================== 内部函数声明 ====================
static void Timer_10ms_Control_Task(void);

/**
 * @brief 定时器0中断服务函数，1ms周期，
 * @details 负责系统的1ms基准定时，调度5ms、10ms、100ms及1s周期任务。
 */
void TIMER_0_INST_IRQHandler(void) {
  switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
  case DL_TIMER_IIDX_ZERO: // TIMG 的加载归零中断标志是 IIDX_ZERO
  {
    ms_ticks++;
    ble_rx_idle_cnt++;

    if (telemetry_pause_ms > 0) {
      telemetry_pause_ms--;
    }
    control_cnt++;

    // 5ms 周期任务处理
    if (control_cnt >= 5) {
      flag_5ms_gyro_read = 1;
      control_cnt = 0;
    }

    // 10ms 周期任务处理
    if (++count_10ms >= 10) {
      count_10ms = 0;
      Timer_10ms_Control_Task();
    }

    // 20ms
    if (++count_100ms >= 100) {
      count_100ms = 0;
    }

    // 1000ms 周期任务处理
    if (++t >= 1000) {
      t = 0;
      flag_1s_printf = 1; // 1秒打印标记，留给主循环执行
      LED3_TOGGLE();
      LED1_TOGGLE();
    }
  } break;

  default:
    break;
  }
}
volatile uint32_t last_time_taken = 0;

float Stand_Hold_Yaw = 0.0f;
uint8_t Dynamic_Angle_Start_Flag = 0; // 全局标志位，用于动态行驶模式的起步控制

/**
 * @brief 10ms 控制任务核心逻辑
 * @details 包括传感器读取、里程积分、PID闭环计算及电机控制输出。
 */

// ==================== 跨模式控制状态机上下文 ====================
float Soft_Basic_Speed = 0.0f; // 软启动基础速度

// Task3 基于时间的 cosine S 曲线斜坡控制 (在 10ms 中断中精确执行)
volatile uint8_t Task3_Ramp_Active = 0;  // 1=斜坡运行中, 0=停止
volatile uint32_t Task3_Ramp_Start = 0;  // 斜坡起始时间戳(ms_ticks)

// Task4 cosine S 曲线斜坡控制 (角度触发刹车)
volatile uint8_t Task4_Ramp_Active = 0;       // 1=斜坡运行中, 0=停止
volatile uint8_t Task4_Brake_Triggered = 0;   // 1=已触发角度刹车, 0=未触发
volatile uint32_t Task4_Ramp_Start = 0;       // 启动时间戳(ms_ticks)
volatile uint32_t Task4_Brake_Start = 0;      // 刹车触发时间戳(ms_ticks)

static uint8_t last_mode = 0;         // 记录上一次的模式状态
static float target_speed_1 = 0.0f;   // 计算得出的左轮目标速度
static float target_speed_2 = 0.0f;   // 计算得出的右轮目标速度
static uint8_t force_pwm_zero = 0;    // 强制滑行冷却标志位
static float target_turn = 0.0f;      // 目标转向差速

/**
 * @brief 航向角闭环模式 (Angle_Mode)
 */
static void Control_Angle_Mode(void) {
  static uint8_t angle_hold_init = 0;
  if (last_mode != Angle_Mode) {
    Dynamic_Angle_Start_Flag = 0;
    angle_hold_init = 0;
  }

  /* 切入角度环时，默认锁定当前车头朝向 */
  if (angle_hold_init == 0) {
    if (JY61P_Data != NULL) {
      Target_Angle = JY61P_Data->total_z;
      angle_hold_init = 1;
    }
    pid_Angle.KiOut = 0;
    pid_Angle.KdOut = 0;
  }

  /* 纯角度环无基础前进速度，仅进行原地转向 */
  Soft_Basic_Speed = 0.0f;

  if (JY61P_Data != NULL && Dynamic_Angle_Start_Flag == 1) {
    target_turn = PID_Calculate(&pid_Angle, JY61P_Data->total_z, Target_Angle);

    /* 输出限幅保护 */
    if (target_turn > (float)pid_Angle.PID_Limit_MAX)
      target_turn = (float)pid_Angle.PID_Limit_MAX;
    if (target_turn < -(float)pid_Angle.PID_Limit_MAX)
      target_turn = -(float)pid_Angle.PID_Limit_MAX;

    /* 差速旋转输出 */
    target_speed_1 = Soft_Basic_Speed - target_turn;
    target_speed_2 = Soft_Basic_Speed + target_turn;
  } else {
    target_speed_1 = 0;
    target_speed_2 = 0;
  }
}

/**
 * @brief 纯角速度闭环模式
 */
static void Control_Gyro_Mode(void) {
  if (last_mode != Gyro_Mode) {
    // 切换模式时阻塞 50ms，确保底层中断状态平稳过渡
    delay_cycles(1600000);

    pid_Gyro.KiOut = 0;
    pid_Gyro.KdOut = 0;
    Target_Gyro = 0.0f;
    // 重置 S 曲线发生器的状态
    target_gyro_ramp = 0.0f;
    soft_target_gyro = 0.0f;
  }

  // 加入“移动4秒后自动驻车”的逻辑
  static uint32_t move_timer_10ms_gyro = 0;
  if (Dynamic_Angle_Start_Flag == 1) {
    move_timer_10ms_gyro++;
    if (move_timer_10ms_gyro >= 400) { // 400 * 10ms = 4秒
      Dynamic_Angle_Start_Flag = 0;    // 强制变回驻车状态
      move_timer_10ms_gyro = 0;
    }
  } else {
    move_timer_10ms_gyro = 0;
  }

  if (Dynamic_Angle_Start_Flag == 1) {
    if (JY61P_Data != NULL) {
      // 1. 获取全局平滑处理后的物理角速度
      float current_gyro_z = JY61P_Data->gyro_z;

      // ================= S型曲线发生器 =================
      float max_accel = 3.0f; // 加速度限制
      if (target_gyro_ramp < Target_Gyro) {
        target_gyro_ramp += max_accel;
        if (target_gyro_ramp > Target_Gyro)
          target_gyro_ramp = Target_Gyro;
      } else if (target_gyro_ramp > Target_Gyro) {
        target_gyro_ramp -= max_accel;
        if (target_gyro_ramp < Target_Gyro)
          target_gyro_ramp = Target_Gyro;
      }

      // 使用一阶滤波柔化斜坡
      soft_target_gyro = soft_target_gyro * 0.70f + target_gyro_ramp * 0.30f;

      // 2. 前馈计算 (Feedforward)
      float turn_ff = soft_target_gyro * Gyro_Feedforward_Coefficient;

      // 3. 积分分离 (Integral Separation)
      if (fabs(soft_target_gyro - Target_Gyro) > 1.0f ||
          fabs(Target_Gyro - current_gyro_z) > 5.0f) {
        pid_Gyro.KiOut = 0.0f; // 强制清零，禁止积分
      }

      // 4. PID 闭环补偿
      float turn_pid_compensate =
          PID_Calculate(&pid_Gyro, current_gyro_z, soft_target_gyro);
      if (turn_pid_compensate > (float)pid_Gyro.PID_Limit_MAX)
        turn_pid_compensate = (float)pid_Gyro.PID_Limit_MAX;
      if (turn_pid_compensate < -(float)pid_Gyro.PID_Limit_MAX)
        turn_pid_compensate = -(float)pid_Gyro.PID_Limit_MAX;

      // 5. 最终输出
      target_turn = turn_ff + turn_pid_compensate;
    } else {
      target_turn = 0.0f;
    }
  } else {
    target_turn = 0.0f;
    pid_Gyro.KiOut = 0.0f;
  }

  // 纯原地差速旋转
  target_speed_1 = -target_turn; // 左轮
  target_speed_2 = target_turn;  // 右轮
}

/**
 * @brief 速度环调参模式 (上位机联动)
 */
static void Control_Speed_Mode(void) {
  static uint8_t mode1_init_delay = 0;
  if (mode1_init_delay == 0) {
    delay_cycles(1600000);
    mode1_init_delay = 1;
  }
  static float speed_mode_target = 0.0f;
  static float speed_mode_intermediate = 0.0f; // 用于生成二阶S曲线的中间变量

  static long long start_pulse1 = 0;
  static long long start_pulse2 = 0;
  static uint32_t start_time = 0;
  static uint8_t is_running = 0;

  static float target_locked_yaw = 0.0f;
  if (Basic_Speed != 0.0f && Speed_Mode_Run_Timer == 0) {
    start_pulse1 = Motor1_Total_Pulse;
    start_pulse2 = Motor2_Total_Pulse;
    start_time = ms_ticks;
    is_running = 1;

    // 【锁头算法 1】
    target_locked_yaw = JY61P_Data->total_z;
    pid_Angle.KiOut = 0;
    pid_Angle.KdOut = 0;
  }

  if (is_running && Basic_Speed != 0.0f) {
    long long diff1 = Motor1_Total_Pulse - start_pulse1;
    long long diff2 = Motor2_Total_Pulse - start_pulse2;
    if (diff1 >= 1040 || diff1 <= -1040 || diff2 >= 1040 || diff2 <= -1040) {
      last_time_taken = ms_ticks - start_time;
      Basic_Speed = 0.0f;
    }
  }

  if (Basic_Speed != 0.0f) {
    if (Speed_Mode_Run_Timer < 180) {
      Speed_Mode_Run_Timer++;
      // 【二阶 S 曲线加速】：起步柔和，中段发力，后段平稳触顶
      speed_mode_intermediate = speed_mode_intermediate * 0.90f + Basic_Speed * 0.10f;
      speed_mode_target = speed_mode_target * 0.90f + speed_mode_intermediate * 0.10f;

      // 【锁头算法 2】
      float yaw_compensate =
          PID_Calculate(&pid_Angle, JY61P_Data->total_z, target_locked_yaw);
      if (yaw_compensate > 20.0f)
        yaw_compensate = 20.0f;
      if (yaw_compensate < -20.0f)
        yaw_compensate = -20.0f;

      target_speed_1 = speed_mode_target - yaw_compensate;
      target_speed_2 = speed_mode_target + yaw_compensate;
    } else {
      // 1.8秒后自动减速（同样使用 S 曲线平滑过渡）
      speed_mode_intermediate = speed_mode_intermediate * 0.90f;
      speed_mode_target = speed_mode_target * 0.90f + speed_mode_intermediate * 0.10f;
      if (speed_mode_target < 1.0f)
        speed_mode_target = 0.0f;

      float yaw_compensate =
          PID_Calculate(&pid_Angle, JY61P_Data->total_z, target_locked_yaw);
      if (yaw_compensate > 10.0f)
        yaw_compensate = 10.0f;
      if (yaw_compensate < -10.0f)
        yaw_compensate = -10.0f;

      target_speed_1 = speed_mode_target - yaw_compensate;
      target_speed_2 = speed_mode_target + yaw_compensate;
    }
  } else {
    Speed_Mode_Run_Timer = 0;
    // 指令停车（平滑 S 曲线刹车）
    speed_mode_intermediate = speed_mode_intermediate * 0.85f;
    speed_mode_target = speed_mode_target * 0.85f + speed_mode_intermediate * 0.15f;
    if (speed_mode_target < 1.0f) {
      speed_mode_target = 0.0f;
      if (is_running) {
        is_running = 0;
      }
    }
    target_speed_1 = speed_mode_target;
    target_speed_2 = speed_mode_target;
  }
}

/**
 * @brief 驻车锁死模式 (上电默认)
 */
static void Control_Stand_Mode(void) {
  static uint8_t hold_init = 0;

  if (last_mode != Stand_Mode) {
    hold_init = 0;
  }

  if (hold_init == 0) {
    if (JY61P_Data != NULL) {
      Stand_Hold_Yaw = JY61P_Data->total_z;
      pid_Stand_Angle.KiOut = 0;
      pid_Stand_Angle.KdOut = 0;
      hold_init = 1;
    }
  }

  // 驻车角度 PD 控制
  float hold_turn = 0.0f;
  if (JY61P_Data != NULL) {
    float angle_error = Stand_Hold_Yaw - JY61P_Data->total_z;
    static float last_angle_error = 0.0f;

    float turn_kp = pid_Stand_Angle.Kp;
    float turn_kd = pid_Stand_Angle.Kd;

    float d_angle_error = angle_error - last_angle_error;
    last_angle_error = angle_error;

    hold_turn = (angle_error * turn_kp) + (d_angle_error * turn_kd);

    if (hold_turn > 400.0f)
      hold_turn = 400.0f;
    if (hold_turn < -400.0f)
      hold_turn = -400.0f;
  }

  target_speed_1 = -hold_turn;
  target_speed_2 = hold_turn;
}

/**
 * @brief 循迹模式 (Run_Mode)
 */
// 软启动重置标志：置 1 时 Control_Run_Mode 会将 intermediate_speed 和
// Soft_Basic_Speed 重置为 Run_Mode_Init_Speed，用于任务三/四从启动速度开始加速
volatile uint8_t Run_Mode_Soft_Reset = 0;
volatile float Run_Mode_Init_Speed = 0.0f;

static void Control_Run_Mode(void) {
  Huidu_Proc(Huidu_Datas);

  // ============ 速度斜坡管理 ============
  // Task3: 基于时间的 cosine S 曲线 (5秒后减速)
  // Task4: 基于时间的 cosine S 曲线 (角度触发减速)
  // 其他模式: 沿用原有二阶低通滤波
  extern uint8_t Task_Mode;
  extern float Task3_Accel_Coeff;   // 语义: 加速时间(秒)
  extern float Task3_Decel_Coeff;   // 语义: 减速时间(秒)
  extern float Task3_Cruise_Speed;
  extern float Task3_Start_Speed;
  extern float Task4_Accel_Coeff;
  extern float Task4_Decel_Coeff;
  extern float Task4_Cruise_Speed;
  extern float Task4_Start_Speed;

  if (Task_Mode == 3 && Task3_Ramp_Active) {
    // ====== Task3 cosine S 曲线 ======
    float elapsed_sec = (ms_ticks - Task3_Ramp_Start) / 1000.0f;

    if (elapsed_sec < 5.0f) {
      // 加速阶段 (0~5秒): 起步速度 -> 巡航速度
      float accel_time = Task3_Accel_Coeff;
      if (accel_time < 0.1f) accel_time = 0.1f;  // 防呆
      if (elapsed_sec < accel_time) {
        float ratio = elapsed_sec / accel_time;
        // cosine S 曲线: 缓起步 -> 平稳加速 -> 缓触顶
        float s = 0.5f * (1.0f - cosf(3.14159265f * ratio));
        Soft_Basic_Speed = Task3_Start_Speed + (Task3_Cruise_Speed - Task3_Start_Speed) * s;
      } else {
        Soft_Basic_Speed = Task3_Cruise_Speed;
      }
    } else {
      // 减速阶段 (5秒后): 巡航速度 -> 0
      float brake_elapsed = elapsed_sec - 5.0f;
      float decel_time = Task3_Decel_Coeff;
      if (decel_time < 0.1f) decel_time = 0.1f;
      if (brake_elapsed < decel_time) {
        float ratio = brake_elapsed / decel_time;
        // cosine S 曲线: 缓起步减速 -> 平稳减速 -> 缓停止
        float s = 0.5f * (1.0f + cosf(3.14159265f * ratio));
        Soft_Basic_Speed = Task3_Cruise_Speed * s;
      } else {
        Soft_Basic_Speed = 0.0f;
      }
    }
  } else if (Task_Mode == 4 && Task4_Ramp_Active) {
    // ====== Task4 cosine S 曲线 (角度触发刹车) ======
    if (!Task4_Brake_Triggered) {
      // 未触发刹车: 加速阶段或巡航阶段
      float elapsed_sec = (ms_ticks - Task4_Ramp_Start) / 1000.0f;
      float accel_time = Task4_Accel_Coeff;
      if (accel_time < 0.1f) accel_time = 0.1f;
      if (elapsed_sec < accel_time) {
        float ratio = elapsed_sec / accel_time;
        float s = 0.5f * (1.0f - cosf(3.14159265f * ratio));
        Soft_Basic_Speed = Task4_Start_Speed + (Task4_Cruise_Speed - Task4_Start_Speed) * s;
      } else {
        Soft_Basic_Speed = Task4_Cruise_Speed;
      }
    } else {
      // 已触发刹车: cosine 减速到0
      float brake_elapsed = (ms_ticks - Task4_Brake_Start) / 1000.0f;
      float decel_time = Task4_Decel_Coeff;
      if (decel_time < 0.1f) decel_time = 0.1f;
      if (brake_elapsed < decel_time) {
        float ratio = brake_elapsed / decel_time;
        float s = 0.5f * (1.0f + cosf(3.14159265f * ratio));
        Soft_Basic_Speed = Task4_Cruise_Speed * s;
      } else {
        Soft_Basic_Speed = 0.0f;
      }
    }
  } else {
    // ====== 其他模式: 原有二阶低通滤波 ======
    static float intermediate_speed = 0.0f;
    if (Run_Mode_Soft_Reset) {
      intermediate_speed = Run_Mode_Init_Speed;
      Soft_Basic_Speed = Run_Mode_Init_Speed;
      Run_Mode_Soft_Reset = 0;
    }
    // 使用固定系数(不依赖Task3/4参数,因这些参数现在是时间而非系数)
    float current_coeff = 0.05f;
    intermediate_speed = intermediate_speed * (1.0f - current_coeff) + Basic_Speed * current_coeff;
    Soft_Basic_Speed = Soft_Basic_Speed * (1.0f - current_coeff) + intermediate_speed * current_coeff;
  }

  static uint16_t auto_stop_timer = 0;
  static float current_run_score = 0;

  if (Tracking_Test_Flag == 1) {
    static float last_score_error = 0;
    if (auto_stop_timer == 0) {
      current_run_score = 0.0f;
      last_score_error = Huidu_Error;
    }

    Basic_Speed = Target_Speed_Test;
    Motors_Enable();
    Turn_PID_Flag = 1;

    auto_stop_timer++;

    float abs_err = (Huidu_Error < 0) ? -Huidu_Error : Huidu_Error;
    float delta_err = Huidu_Error - last_score_error;
    float abs_delta = (delta_err < 0) ? -delta_err : delta_err;

    current_run_score += (abs_err + abs_delta * 5.0f);
    last_score_error = Huidu_Error;

    if (auto_stop_timer >= 700) {
      Tracking_Test_Flag = 0;
      Tuning_State = 0;

      char msg[64];
      sprintf(msg, "[AUTO_SCORE]:%d\r\n", (int)current_run_score);
      BLE_send_String((unsigned char *)msg);
    }
  } else {
    auto_stop_timer = 0;
    // =========================================================================
    // [BUG FIX]: Removed forceful override of Basic_Speed, Turn_PID_Flag, and Motors_Disable.
    // These are managed by Task_Scheduler in task.c. Overriding them here causes
    // the car to ignore Task_1 and remain stationary.
    // =========================================================================
  }

  if (Turn_PID_Flag == 1) {
    // 【直道防震荡保护】：死区积分清零 (Integral Separation)
    // 如果误差很小（即黑线在最中间的两个探头0.5处徘徊），
    // 强制清空积分项！这彻底杜绝了直道上的“蛇形走位”震荡。
    // 只有当真正进入弯道，误差放大到 1.5 以上时，积分项才会发力把车头拉回来！
    if (fabs(Huidu_Error) < 0.8f) {
        pid_Turn.KiOut = 0.0f;
    }
    
    // 恢复经典的线性 PID 算法。针对【电赛H题：车载平衡滚球】的平滑循迹需求优化！
    // H题赛道弯道半径高达 0.5米，是非常平缓的弯道，极度忌讳猛烈打方向，否则钢球必掉！
    float raw_target_turn = PID_Calculate(&pid_Turn, Huidu_Error, 0);
    
    // 取消为滚球平衡任务特化的二阶极度平滑EMA（0.85/0.15延迟极大）
    // 恢复常规循迹的低延迟转向，消灭“走到快脱线才拉回”
    static float intermediate_turn = 0.0f;
    intermediate_turn = intermediate_turn * 0.4f + raw_target_turn * 0.6f;
    target_turn = target_turn * 0.4f + intermediate_turn * 0.6f;
    
    // 取消滚球平衡任务特化的高强度减速 (1.0f)，消除严重的“停顿感”
    // 降至 0.3f，让内侧减速的同时外侧依然可以加速转向
    float current_basic_speed = Soft_Basic_Speed;
    float speed_drop = fabs(target_turn) * 0.3f; 
    current_basic_speed -= speed_drop;
    
    // 【致命漏洞修复：移除底层牵引力强制托底】
    // 之前代码强制限制 current_basic_speed > 70。在急弯时，这个托底导致 speed_drop 无法继续抵消外侧轮的转向叠加值，
    // 从而导致外侧轮的实际速度超过了巡航速度，产生了您感受到的“小幅突增（闯动）”。
    // 现在彻底放开，允许内侧轮反转，外侧轮速度被数学上完美“锁死”在巡航速度，绝不突增！
    
    // 将平滑后的转向指令叠加入目标速度，由强悍的速度环去闭环执行！
    target_speed_1 = current_basic_speed - target_turn;
    target_speed_2 = current_basic_speed + target_turn;
  } else {
    target_turn = 0.0f; // [BUG FIX] 必须清空历史转向，防止关闭循迹后小车继续画圆
    target_speed_1 = Soft_Basic_Speed - target_turn;
    target_speed_2 = Soft_Basic_Speed + target_turn;
  }
}

/**
 * @brief 附加逻辑：悬空测试模式
 */
/**
 * @brief 定长位移闭环模式 (Distance_Mode)
 */
static void Control_Distance_Mode(void) {
  static uint8_t init_flag = 0;
  static float start_distance = 0.0f;
  static float locked_yaw = 0.0f;

  /* 用于存储矢量投影后的有效直线位移 */
  static float projected_distance = 0.0f;
  static float last_raw_distance = 0.0f;

  if (last_mode != Distance_Mode) {
    init_flag = 0;
    pid_Distance.KiOut = 0;
    pid_Distance.KdOut = 0;
    pid_Angle.KiOut = 0;
    pid_Angle.KdOut = 0;
    Dynamic_Angle_Start_Flag = 0;
  }

  if (init_flag == 0) {
    if (JY61P_Data != NULL) {
      locked_yaw = JY61P_Data->total_z;
      start_distance = Measure_Distance;
      last_raw_distance = Measure_Distance;
      projected_distance = 0.0f;
      init_flag = 1;
    }
  }

  float active_target_distance = 0.0f;
  if (Dynamic_Angle_Start_Flag == 1) {
    active_target_distance = Target_Distance;
  }

  if (init_flag == 1) {
    /* =========================================================================
     * [1] 航位推算 (Odometry Projection)
     * 将编码器的弧线距离，沿初始锁定的航向角投影到理想直线上
     * =========================================================================
     */
    float ds = Measure_Distance - last_raw_distance;
    last_raw_distance = Measure_Distance;

    float delta_yaw = JY61P_Data->total_z - locked_yaw;
    float dx = ds * cosf(delta_yaw * 3.14159265f / 180.0f);

    projected_distance += dx;
    float current_dist = projected_distance;

    /* =========================================================================
     * [2] 距离环 PID 控制
     * =========================================================================
     */
    float dist_out = 0.0f;
    if (Dynamic_Angle_Start_Flag == 1) {
      dist_out =
          PID_Calculate(&pid_Distance, current_dist, active_target_distance);
      if (dist_out > MAX_ALLOWED_SPEED_MM_S)
        dist_out = MAX_ALLOWED_SPEED_MM_S;
      if (dist_out < -MAX_ALLOWED_SPEED_MM_S)
        dist_out = -MAX_ALLOWED_SPEED_MM_S;
    }

    /* =========================================================================
     * [3] 对称梯形加减速规划 (Symmetric Slew Rate Limiter)
     * =========================================================================
     */
    float speed_err = dist_out - Soft_Basic_Speed;

    float accel_step = 5.0f;
    float brake_step = 5.0f;

    if (Soft_Basic_Speed >= 0.0f) {
      if (speed_err > accel_step) {
        Soft_Basic_Speed += accel_step;
      } else if (speed_err < -brake_step) {
        Soft_Basic_Speed -= brake_step;
      } else {
        Soft_Basic_Speed = Soft_Basic_Speed * 0.90f + dist_out * 0.10f;
      }
    } else {
      if (speed_err < -accel_step) {
        Soft_Basic_Speed -= accel_step;
      } else if (speed_err > brake_step) {
        Soft_Basic_Speed += brake_step;
      } else {
        Soft_Basic_Speed = Soft_Basic_Speed * 0.90f + dist_out * 0.10f;
      }
    }

    /* =========================================================================
     * [4] 航向修正闭环与差速输出
     * =========================================================================
     */
    float yaw_compensate =
        PID_Calculate(&pid_Angle, JY61P_Data->total_z, locked_yaw);
    if (yaw_compensate > 150.0f)
      yaw_compensate = 150.0f;
    if (yaw_compensate < -150.0f)
      yaw_compensate = -150.0f;

    target_speed_1 = Soft_Basic_Speed - yaw_compensate;
    target_speed_2 = Soft_Basic_Speed + yaw_compensate;
  }
}



static void Apply_Test_Speed_Mode(void) {
  static float Test_Soft_Speed = 0.0f;
  static float locked_yaw = 0.0f;
  static uint8_t gyro_lock_init = 0;

  if (Test_Speed_Mode == 1) {
    Test_Soft_Speed = Test_Soft_Speed * 0.90f + Target_Speed_Test * 0.10f;

    if (Target_Speed_Test != 0 && JY61P_Data != NULL) {
      if (gyro_lock_init == 0) {
        locked_yaw = JY61P_Data->total_z;
        gyro_lock_init = 1;
        pid_Gyro.KiOut = 0;
        pid_Gyro.PID_Out = 0;
        pid_Gyro.Error = 0;
      }
      float turn_comp =
          PID_Calculate(&pid_Gyro, JY61P_Data->total_z, locked_yaw);

      if (turn_comp > 15.0f)
        turn_comp = 15.0f;
      if (turn_comp < -15.0f)
        turn_comp = -15.0f;

      target_speed_1 = Test_Soft_Speed - turn_comp;
      target_speed_2 = Test_Soft_Speed + turn_comp;
    } else {
      gyro_lock_init = 0;
      target_speed_1 = Test_Soft_Speed;
      target_speed_2 = Test_Soft_Speed;
    }
  } else {
    Test_Soft_Speed = 0.0f;
    gyro_lock_init = 0;
  }
}

uint8_t Task2_Trajectory_Enabled = 0;
uint8_t Task2_Restart_Flag = 0;

/**
 * @brief 第三问控制模式 (Ball_Task2_Mode) 轨迹控制
 */
static void Control_Ball_Task2_Mode(void) {
    static uint8_t init_flag = 0;
    static uint32_t state_timer = 0;
    static uint8_t task_step = 0;
 
    // 轨迹模式状态 (0→+5cm→-5cm稳定)
    static uint8_t traj_step = 0;       // 0=未启动 1=去+5cm 2=去-5cm 3=稳定
    static uint32_t traj_timer = 0;     // 轨迹阶段计时(10ms计数)
 
    if (last_mode != Ball_Task2_Mode || Task2_Restart_Flag) {
        init_flag = 0;
        Task2_Restart_Flag = 0;
    }
 
    if (init_flag == 0) {
        PID_Reset(&pid_Ball_Beam);
        state_timer = 0;
        task_step = 0;
        if (Task2_Trajectory_Enabled) {
            traj_step = 1;              // 从 0→+5cm 开始
            traj_timer = 0;
        } else {
            traj_step = 0;
        }
        init_flag = 1;
    }
 
    extern volatile int32_t Ball_Pos_X;
    float current_pos = Ball_Pos_X;
 
    // ================== 平滑滤波与防抖处理 ==================
    static float filtered_pos = 320.0f;
    if (init_flag == 1 && state_timer == 0) {
        filtered_pos = current_pos; // 首次初始化
        state_timer = 1;            // 标记已完成初始化，避免重复赋值
    } else {
        filtered_pos = 0.4f * filtered_pos + 0.6f * current_pos;
    }
 
    // ================== 目标位置设置 ==================
    // 本项目坐标换算: 25cm = 700 pixels -> 1cm = 28 pixels
    // 基准位置由 K230 通过 FF 帧设置 (Ball_Target_Set_Pos), 默认 295
    // +5cm = 基准 + 140, -5cm = 基准 - 140
    extern volatile float Ball_Target_Set_Pos;
    float base_pos = Ball_Target_Set_Pos;
    float pos_plus5 = base_pos + 140.0f;   // +5cm
    float pos_minus5 = base_pos - 140.0f;  // -5cm

    if (Task2_Trajectory_Enabled) {
        // 轨迹模式: 0→+5cm→-5cm稳定, 总时间≤5s
        switch (traj_step) {
            case 1: // 阶段1: 0→+5cm
                pid_Ball_Beam.Target = pos_plus5;
                traj_timer++;
                // 到达判定: 提前0.5cm触发(目标-14像素)补偿滤波延迟
                if (filtered_pos >= (pos_plus5 - 14.0f) || traj_timer >= 250) {
                    traj_step = 2;
                    traj_timer = 0;
                }
                break;
            case 2: { // 阶段2: +5cm→-5cm (斜坡过渡防回拉过猛+预刹车防过冲)
                traj_timer++;
                // 前1500ms分段线性插值: 主斜坡1000ms + 慢速收尾500ms
                float mid_pos = (pos_plus5 + pos_minus5) / 2.0f; // 中间点
                if (traj_timer <= 100) {
                    // 0-1000ms: +5cm→中间点 线性下降(主斜坡)
                    pid_Ball_Beam.Target = pos_plus5 - (pos_plus5 - mid_pos) * traj_timer / 100;
                } else if (traj_timer <= 150) {
                    // 1000-1500ms: 中间点→-5cm 慢速收尾(预刹车)
                    pid_Ball_Beam.Target = mid_pos - (mid_pos - pos_minus5) * (traj_timer - 100) / 50;
                } else {
                    pid_Ball_Beam.Target = pos_minus5;    // -5cm
                }
                // 到达判定: 真实位置进入-5cm±1.0cm 或 超时3.2s
                if (filtered_pos <= (pos_minus5 + 25.0f) || traj_timer >= 320) {
                    traj_step = 3;
                    traj_timer = 0;
                }
                break;
            }
            case 3: // 阶段3: 稳定在-5cm
                pid_Ball_Beam.Target = pos_minus5;
                break;
            default:
                pid_Ball_Beam.Target = base_pos;   // 默认基准位置
                break;
        }
    } else {
        // 定点模式: 停在基准位置 (K230设置则用其值, 否则用默认295)
        extern uint8_t bt_user_override;
        if (!bt_user_override) {
            pid_Ball_Beam.Target = base_pos;
        }
    }
 
    float output = 0.0f;
    // 极小死区(1像素)避免电机因像素抖动而抽搐
    if (fabs(filtered_pos - pid_Ball_Beam.Target) < 1.0f) {
        output = 0.0f;
    } else {
        output = PID_Calculate(&pid_Ball_Beam, filtered_pos, pid_Ball_Beam.Target);
    }
 
    // 覆盖 Gimbal_Speed_Y，阻断视觉下发的 BB 帧影响
    Gimbal_Speed_Y = (int16_t)-output; // 负号保证正常负反馈
}

/**
 * @brief 终极合并：速度环 PID 计算与 PWM 下发
 */
static void Calculate_And_Apply_Speed_PID(void) {
  // 统一限幅
  if (target_speed_1 > MAX_ALLOWED_SPEED_MM_S)
    target_speed_1 = MAX_ALLOWED_SPEED_MM_S;
  if (target_speed_1 < -MAX_ALLOWED_SPEED_MM_S)
    target_speed_1 = -MAX_ALLOWED_SPEED_MM_S;
  if (target_speed_2 > MAX_ALLOWED_SPEED_MM_S)
    target_speed_2 = MAX_ALLOWED_SPEED_MM_S;
  if (target_speed_2 < -MAX_ALLOWED_SPEED_MM_S)
    target_speed_2 = -MAX_ALLOWED_SPEED_MM_S;

  MEASURE_MOTORS_SPEED();
  float pwm1 = 0, pwm2 = 0;

  if (Car_Mode == AutoTune_Mode) {
    AutoTune_Process(&pwm1, &pwm2);
  } else {
    if (force_pwm_zero == 1) {
      pwm1 = 0;
      pwm2 = 0;
    } else {
      float original_kd_1 = pid_Motor1_Speed.Kd;
      float original_kd_2 = pid_Motor2_Speed.Kd;

      if (Car_Mode == Gyro_Mode) {
        if (target_speed_1 >= -150.0f && target_speed_1 <= 150.0f)
          pid_Motor1_Speed.Kd = 0.0f;
        if (target_speed_2 >= -150.0f && target_speed_2 <= 150.0f)
          pid_Motor2_Speed.Kd = 0.0f;
      }

      pwm1 = PID_Calculate(&pid_Motor1_Speed, Motor1_Speed, target_speed_1);
      pwm2 = PID_Calculate(&pid_Motor2_Speed, Motor2_Speed, target_speed_2);

      pid_Motor1_Speed.Kd = original_kd_1;
      pid_Motor2_Speed.Kd = original_kd_2;
    }
  }

  if (Max_Speed_Test_Flag == 1) {
    pwm1 = 9999;
    pwm2 = 9999;
  }

  SET_MOTORS_SPEED((int)pwm1, (int)pwm2);
}

/**
 * @brief 10ms 控制任务核心逻辑（重构后的状态机调度器）
 * @details 负责任务积分、传感器更新，并将控制权分发给各个独立模式。
 */
static void Timer_10ms_Control_Task(void) {
  static uint8_t cnt_50ms = 0;

  // 20ms 标志位生成 (极限提速 50Hz)
  if (++cnt_50ms >= 2) {
    cnt_50ms = 0;
    flag_50ms_lcd = 1;
    if (telemetry_pause_ms == 0) {
      flag_50ms_telemetry = 1;
    }
  }

  // 任务时间及里程积分
  if (Task1_Time_flag == 1) {
    Task1_Time_Sec += 0.01f;
    Task1_Dist += (Motor1_Speed + Motor2_Speed) / 2.0f;
  } else if (Task2_Time_flag == 1) {
    Task2_Time_Sec += 0.01f;
    Task2_Dist += (Motor1_Speed + Motor2_Speed) / 2.0f;
  }

  // 处理视觉下发的绝对位置模式数据 (解耦，防止在串口接收中断中发送导致中断风暴卡死)
  // 【修复】：彻底屏蔽此段遗留代码。K230_Target_Pos 是像素坐标(320左右)，直接作为绝对脉冲发送给 PD42S1(初始值22300) 会导致上电时电机剧烈偏离初始位置。
  if (K230_New_Pos_Flag == 1) {
    /*
    if (Car_Mode != Ball_Task2_Mode) {
      static int32_t last_pos_x = -999999;
      static int32_t last_pos_y = -999999;
      if (K230_Target_Pos_X != last_pos_x) {
        uint8_t dir_x = (K230_Target_Pos_X >= 0) ? 0 : 1;
        uint32_t pulse_x = (K230_Target_Pos_X >= 0) ? K230_Target_Pos_X : -K230_Target_Pos_X;
        g_smd_target_uart = 1;
        smd_pos_mode(1, dir_x, 5, 200, pulse_x);
        last_pos_x = K230_Target_Pos_X;
      }
      if (K230_Target_Pos_Y != last_pos_y) {
        uint8_t dir_y = (K230_Target_Pos_Y >= 0) ? 0 : 1;
        uint32_t pulse_y = (K230_Target_Pos_Y >= 0) ? K230_Target_Pos_Y : -K230_Target_Pos_Y;
        g_smd_target_uart = 2;
        smd_pos_mode(1, dir_y, 5, 200, pulse_y);
        last_pos_y = K230_Target_Pos_Y;
      }
    }
    */
    K230_New_Pos_Flag = 0;
  }

  // 传感器状态更新
  Key_Read();
  Huidu_Read();

  // 状态机分发 (必须先于 Car_Set_Speed_Unified 执行，
  // 因为 Control_Ball_Task2_Mode 会设置 Gimbal_Speed_Y，
  // 如果先调用 Car_Set_Speed_Unified 会用到上一周期的旧值)
  switch (Car_Mode) {
  case Angle_Mode:
    Control_Angle_Mode();
    break;
  case Gyro_Mode:
    Control_Gyro_Mode();
    break;
  case Speed_Mode:
    Control_Speed_Mode();
    break;
  case Stand_Mode:
    Control_Stand_Mode();
    break;
  case Distance_Mode:
    Control_Distance_Mode();
    break;
  case AutoTune_Mode: /* 自动整定，底层处理 */
    break;
  case Ball_Task2_Mode:
    Control_Run_Mode(); // 允许底盘受 Basic_Speed 控制 (任务四合并所需)
    Control_Ball_Task2_Mode();
    break;
  default:
    Control_Run_Mode();
    break;
  }

  // 云台/步进电机控制下发 (移到状态机之后，确保任务三的PID输出能立即生效)
  if (Task_Mode != 0) {
    Car_Set_Speed_Unified(Gimbal_Speed_X, Gimbal_Speed_Y);
  }

  // 测试模式速度覆盖
  Apply_Test_Speed_Mode();

  // 最终合成 PID 并下发 PWM
  Calculate_And_Apply_Speed_PID();

  last_mode = Car_Mode;
}

/**
 * @brief 定时器1中断服务函数，3ms周期，
 */
void TIMER_1_INST_IRQHandler(void) {
  switch (DL_TimerG_getPendingInterrupt(TIMER_1_INST)) {
  case DL_TIMER_IIDX_ZERO:
    break;
  default:
    break;
  }
}

/**
 * @brief 定时器2中断服务函数，10us周期，
 * @note 触发频率不可过高，防止中断过载
 */
void TIMER_2_INST_IRQHandler(void) {
  switch (DL_TimerG_getPendingInterrupt(TIMER_2_INST)) {
  case DL_TIMER_IIDX_LOAD:
    break;
  default:
    break;
  }
}

/**
 * @brief 系统中断配置初始化
 * @details 清除并使能所需的所有外设（串口、定时器、编码器等）中断。
 */
void NVIC_EnableIRQ_Init(void) {
  // 清除串口中断标志
  NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(UART_MOTOR_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(UART_MOTOR_2_INST_INT_IRQN);

  // 使能串口中断
  NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
  NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
  NVIC_EnableIRQ(UART_MOTOR_INST_INT_IRQN);
  NVIC_EnableIRQ(UART_MOTOR_2_INST_INT_IRQN);

  // 清除定时器中断标志
  NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(TIMER_1_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(TIMER_2_INST_INT_IRQN);

  // 使能定时器中断
  NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
  NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);
  NVIC_EnableIRQ(TIMER_2_INST_INT_IRQN);

  // 启动定时器A计数
  DL_TimerG_startCounter(TIMER_0_INST);

  // 编码器 (PORTA) 中断使能
  NVIC_EnableIRQ(GPIOA_INT_IRQn);

  // 陀螺仪中断使能（如果需要）
  // NVIC_EnableIRQ(MPU6050_INT_IRQN);
}
