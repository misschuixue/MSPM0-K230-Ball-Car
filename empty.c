/**
 * @file empty.c
 * @brief 系统主控中枢与任务调度引擎 (Main)
 * @details 负责外设初始化、状态机流转及各个时间基准下循环任务的分发。
 * @author WILLiam
 * @date 2026-07-10
 */

/* ==================== 包含文件 ==================== */
// 芯片与板级抽象层 (BSP)
#include "delay.h"
#include "ti_msp_dl_config.h"

// 核心业务与状态机头文件
#include "main.h"

// 传感器与外设驱动层
#include "Encoder.h"
#include "bsp_gyro.h"
#include "bsp_hc05.h"
#include "gw_gray.h"
#include "hw_lcd.h"
#include "key.h"
#include "motor_ctrl.h"
#include "timer.h"
#include "usart.h"

// 中间件与协议层
#include "app_protocol.h"
#include "protocol.h"
#include "task.h"

// UI 交互与显示层
#include "app_lcd.h"
#include "bmp.h"

/* ==================== 运动学闭环与系统状态集 (定义) ==================== */
uint8_t Car_Mode = Run_Mode; // 系统有限状态机阶段 (默认开启循迹模式 Run_Mode)
uint8_t OLED_View_Select = 1; // OLED 屏幕默认交互页面指针

// 运动学闭环目标期望值 (Setpoints)
float Basic_Speed = 300;   // 前向基础牵引速度 (mm/s)
float Target_Distance = 0; // 目标位移量 (mm)
float Target_Gyro = 0;     // 目标自旋角速度 (度/s)
float Target_Angle = 0;    // 目标航向角 (度)

// 全局传感器状态缓存
Gyro_Struct *JY61P_Data; // 陀螺仪位姿状态读取缓存区

/* ==================== 外部跨模块状态引入 ==================== */
// 定时器中断标志位
extern volatile uint8_t flag_5ms_gyro_read;
extern uint8_t flag_1s_printf;

// 电机与调参状态
extern uint8_t Test_Speed_Mode;
extern float Target_Speed_Test;
extern volatile long long Motor1_Total_Pulse;
extern volatile long long Motor2_Total_Pulse;
extern uint8_t Set_Motor_Param_Select;

// 系统时基
extern volatile uint32_t last_time_taken;
extern volatile uint32_t ms_ticks;

// 跨模块函数调用
extern void BLE_send_String(unsigned char *str);
extern uint8_t jy61p_Init_Process(void);
extern void AutoTune_Init(void);
extern void Motors_Enable(void);

/* ==================== 本地私有任务声明 ==================== */
void LCD_Init(void);
void PD42S1_Init(void);
static void Main_Loop_5ms_Task(void);
static void Main_Loop_1s_Task(void);
static void Main_Loop_Background_Task(void);

/* ==================== 业务逻辑与任务分发引擎 ==================== */

/**
 * @brief 5ms 核心实时控制循环 (由定时器中断驱动)
 */
static void Main_Loop_5ms_Task(void) {
  if (flag_5ms_gyro_read) {
    flag_5ms_gyro_read = 0;

    // 幂等读取状态缓存：在此统一读取一次陀螺仪姿态数据
    JY61P_Data = get_angle();
  }
}

/**
 * @brief 1s 低频状态汇报与遥测循环 (由定时器中断驱动)
 */
static void Main_Loop_1s_Task(void) {
  if (flag_1s_printf) {
    char msg[128];
    int m1_int, m1_dec, m2_int, m2_dec;

    flag_1s_printf = 0;

    // --- 需求：每隔1秒打印当前步进电机2的位置到蓝牙 ---
    // 任务三期间禁用位置查询，防止主循环的smd_read_pos与10ms中断里的
    // smd_pos_mode竞争UART2导致帧错乱、电机驱动器收到残帧后失能
    if (Car_Mode != Ball_Task2_Mode) {
      extern uint8_t g_smd_target_uart;
      g_smd_target_uart = 2; // 路由切到串口2 (步进电机2)
      smd_read_pos(1);       // 电机地址一般为 1 (广播触发查询)
    }

    // 仅在调参模式或驻车模式下打印遥测数据，避免高频 I/O 阻塞任务模式的主循环
    if (Car_Mode == 1 || Car_Mode == 3 || Car_Mode == Stand_Mode) {
      sprintf(msg,
              "[DEBUG] 1s: M_Sel=%d, Basic=%.1f, P=%.2f, I=%.2f, D=%.2f\r\n",
              Set_Motor_Param_Select, Basic_Speed, pid_Motor1_Speed.Kp,
              pid_Motor1_Speed.Ki, pid_Motor1_Speed.Kd);
      BLE_send_String((unsigned char *)msg);

      m1_int = (int)Motor1_Speed;
      m1_dec = (int)(Motor1_Speed * 10) % 10;
      if (m1_dec < 0)
        m1_dec = -m1_dec;

      m2_int = (int)Motor2_Speed;
      m2_dec = (int)(Motor2_Speed * 10) % 10;
      if (m2_dec < 0)
        m2_dec = -m2_dec;

      sprintf(msg,
              "[Telemetry] M1_Pulse: %d | M2_Pulse: %d (Speed: %d.%d | %d.%d) "
              "[Time: %d ms]\r\n",
              (int)Motor1_Total_Pulse, (int)Motor2_Total_Pulse, m1_int, m1_dec,
              m2_int, m2_dec, (int)last_time_taken);
      BLE_send_String((unsigned char *)msg);
    }
  }
}

/**
 * @brief 非实时后台综合任务池 (放置于 while(1) 轮询执行)
 */
static void Main_Loop_Background_Task(void) {
  KEY_PROC();
  LCD_Show_Proc();
  receiving_process();   // 野火上位机解析引擎
  Protocol_Datas_Proc(); // 野火上位机数据发送 (内含限速)

  // 核心状态机及任务调度流转分发
  Task_Scheduler();

  // 蓝牙数据接收处理 (配合上送遥测数据)
  Receive_Bluetooth_Data();
}

/**
 * @brief 硬件初始化及任务主入口
 */
int main(void) {
  SYSCFG_DL_init();      // TI底层外设初始化
  NVIC_EnableIRQ_Init(); // 初始化系统中断及总线优先级

  // 硬件上电冷却期 (当前注释掉，可按需开启以防止初始化风暴)
  // delay_ms(1000);

  Bluetooth_Init();
  char msg[64];
  sprintf(msg, "Gyro Init Start...\r\n");
  BLE_send_String((unsigned char *)msg);

  uint32_t init_start_time = ms_ticks;

  // 非阻塞式陀螺仪初始化 (后续任务分发会处理)
  // while (jy61p_Init_Process() == 0) { ... }

  sprintf(msg, "Gyro Init Done! Took: %d ms\r\n",
          (int)(ms_ticks - init_start_time));
  BLE_send_String((unsigned char *)msg);

  // 初始化其它板级外设
  LCD_Init();  
  PD42S1_Init(); 
  protocol_init(); // 初始化野火协议环形缓冲区

  AutoTune_Init();
  // =========================================================================
  // [BUG FIX]: Removed Motors_Enable() here. It caused a 50ms window where 
  // motors were active before Task_Scheduler could disable them in the main loop,
  // causing the car to jump forward on power-up.
  // =========================================================================

  // 挂载三大循环主轴
  while (1) {
    Main_Loop_5ms_Task();
    Main_Loop_1s_Task();
    Main_Loop_Background_Task();
  }
}

/**
 * @brief LCD 屏幕及背光初始化任务
 */
void LCD_Init(void) {
  lcd_init();
  LCD_Fill(0, 0, LCD_W, LCD_H, BLACK); // 清为黑屏
  LCD_BLK_Set();                       // 打开背光
}

/**
 * @brief PD42S1 闭环步进电机内部运行参数固化配置
 */
void PD42S1_Init(void) {
  /* ==================================================================== */
  /* ================== 电机内部参数固化配置 ============================ */
  /* ==================================================================== */

  // 【配置电机 1】
  g_smd_target_uart = 1; // 路由切到串口1
  clear_uart_rx_buffer(1);
  smd_set_mode(1, 1); // 设置通信位置模式
  handle_ack(1, 50);

  clear_uart_rx_buffer(1);
  smd_param_save(1); // 保存到内部Flash
  handle_ack(1, 50);

  // 闭环步进电机 PID 参数配置 (极小积分 + 强阻尼，防止积分饱和与超调)
  clear_uart_rx_buffer(1);
  smd_set_pos_pid(1, 1200, 2, 1000);
  handle_ack(1, 50);

  // 提高运行/保持电流至 1200mA，增强物理握力
  clear_uart_rx_buffer(1);
  smd_set_pos_torque(1, 1200);
  handle_ack(1, 50);

  // 设置位置模式：加速度 80，最大速度 5，目标脉冲 10500
  clear_uart_rx_buffer(1);
  smd_pos_mode(1, 0, 5, 80, 10500);
  handle_ack(1, 50);

  // 【配置电机 2】
  g_smd_target_uart = 2; // 路由切到串口2
  clear_uart_rx_buffer(2);
  smd_set_mode(1, 1); // 同样强制切换到通信位置模式
  handle_ack(2, 50);

  clear_uart_rx_buffer(2);
  smd_param_save(1);
  handle_ack(2, 50);

  // 同样增加电机2闭环参数
  clear_uart_rx_buffer(2);
  smd_set_pos_pid(1, 1200, 2, 1000);
  handle_ack(2, 50);

  // 同样提高电机2电流
  clear_uart_rx_buffer(2);
  smd_set_pos_torque(1, 1200);
  handle_ack(2, 50);

  // 设置电机2位置参数
  clear_uart_rx_buffer(2);
  extern int32_t PD42S1_Motor2_Pos;
  smd_pos_mode(1, 0, 5, 10, PD42S1_Motor2_Pos);
  handle_ack(2, 50);
}
