#include "key.h"
#include "gw_gray.h"
#include "motor_ctrl.h"
#include "process_frame.h"
#include "smd.h"
#include "task.h"
#include "timer.h"
#include <stdio.h>

// ==================== 提取的外部变量声明 ====================
extern uint8_t Tuning_Mode;
extern uint8_t Tuning_Cursor;
extern uint8_t OLED_View_Select;
extern pid_t pid_Turn;
extern void BLE_send_String(unsigned char *str);

// 结构体声明
KEY Key[KEY_Number];

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------外部变量声明--------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
/* 在 key.c 顶部引入外部变量 */
extern float Target_Angle;
extern float Basic_Speed;
extern pid_t pid_Turn;       // 引入转向/循迹 PID 结构体
extern uint8_t Task_1_State; // 引入任务1状态机
extern uint8_t Task_2_State; // 引入任务2状态机
extern uint8_t Task_3_State; // 引入任务3状态机
extern uint8_t flag_state;   // 引入任务防误触/定点状态标志位

/*-------------------------------------------------------------------------------------------*/
/*-----------------------按键读取函数（需要放入 10ms
 * 中断内进行扫描）--------------------------*/
/*-------------------------------------------------------------------------------------------*/
void Key_Read(void) {
  // 添加读取按键
  Key[0].Down_State = KEY1; // 默认按下为0
  Key[1].Down_State = KEY2;
  Key[2].Down_State = KEY3;

  for (int i = 0; i < KEY_Number; i++) {
    switch (Key[i].Judge_State) {
    case 0:
      if (Key[i].Down_State == 0) {
        Key[i].Judge_State = 1;
        Key[i].Down_Time = 0;
      } else {
        Key[i].Judge_State = 0;
      }
      break;
    case 1:
      if (Key[i].Down_State == 0) {
        Key[i].Judge_State = 2;
      } else {
        Key[i].Judge_State = 0;
      }
      break;
    case 2:
      if ((Key[i].Down_State == 1) && (Key[i].Down_Time <= 70)) {
        if (Key[i].Double_Time_EN == 0) {
          Key[i].Double_Time_EN = 1;
          Key[i].Double_Time = 0;
        } else {
          Key[i].Double_Flag = 1;
          Key[i].Double_Time_EN = 0;
        }
        Key[i].Judge_State = 0;
      } else if ((Key[i].Down_State == 1) && (Key[i].Down_Time > 70)) {
        Key[i].Long_Flag = 1;
        Key[i].Judge_State = 0;
      } else {
        Key[i].Down_Time++;
      }
      break;
    }
    if (Key[i].Double_Time_EN == 1) {
      Key[i].Double_Time++;
      if (Key[i].Double_Time >= 35) {
        Key[i].Short_Flag = 1;
        Key[i].Double_Time_EN = 0;
      }
    }
  }
}

/*-------------------------------------------------------------------------------------------*/
/*-------------------------------------按键处理函数------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/
static uint8_t is_motor_running = 0;

uint8_t Tracking_Test_Flag = 0;
uint8_t Tuning_State = 2; // 0:WAIT, 1:RUN, 2:READY

void KEY_PROC(void) {
  static uint8_t first_init = 0;
  if (first_init == 0) {
    // 强制将按键电平的初始内存值设为 1（松手状态）
    Key[0].Down_State = 1;
    Key[1].Down_State = 1;
    Key[2].Down_State = 1;
    first_init = 1;
    return; // 直接退出第一次错乱的循环，等待定时器读取真实的引脚状态
  }

  /* ============================================== */
  /* =================== 长按逻辑 =================== */
  /* ============================================== */
  // Key[0].Down_State == 0 表示 KEY1 正在被按下
  if (Key[0].Down_State == 0) {
    // 确保只有在“刚按下”的那一瞬间发送一次启动指令
    if (is_motor_running == 0) {
      is_motor_running = 1;
    }
  } else if (Key[1].Down_State == 0) {
    // 确保只有在“刚按下”的那一瞬间发送一次启动指令
    if (is_motor_running == 0) {
      is_motor_running = 1;
    }
  }
  // Key[0].Down_State != 0 表示 KEY1 已被松开
  else {
    // 确保只有在“刚松开”的那一瞬间发送一次停止指令
    if (is_motor_running == 1) {
      is_motor_running = 0;
    }
  }

  /* ================= 引入脱机调参系统变量 ================= */
  /* ======================================================= */

  /* ----- 短按处理 ----- */
  if (Key[0].Short_Flag == 1) // KEY1短按：执行当前选定的任务
  {
    extern volatile uint32_t ms_ticks;
    extern uint8_t Task_Select;
    Task_Mode = Task_Select;
    
    // 根据选择的任务启动对应的状态机
    if (Task_Select == 1) Task_1_State = 1;
    else if (Task_Select == 2) Task_2_State = 1;
    else if (Task_Select == 3) {
      extern uint8_t Task_3_State;
      Task_3_State = 1;
    }
    else if (Task_Select == 4) {
      extern uint8_t Task_4_State;
      Task_4_State = 1;
    }

    count_10ms = 0;
    Key[0].Short_Flag = 0;
  } else if (Key[1].Short_Flag == 1) // KEY2短按：循环切换任务
  {
    extern uint8_t Task_Select;
    Task_Select++;
    if (Task_Select > 4) {
      Task_Select = 1;
    }
    Key[1].Short_Flag = 0;
  } else if (Key[2].Short_Flag == 1) // KEY3短按：加100微调
  {
    extern int32_t PD42S1_Motor2_Pos;
    extern void BLE_send_String(unsigned char *str);
    extern uint8_t g_smd_target_uart;
    
    PD42S1_Motor2_Pos += 100;
    g_smd_target_uart = 2;
    clear_uart_rx_buffer(2);
    smd_pos_mode(1, 0, 5, 80, PD42S1_Motor2_Pos);
    handle_ack(2, 50);
    
    char buf[32];
    sprintf(buf, "Motor2 Pos: %d\r\n", (int)PD42S1_Motor2_Pos);
    BLE_send_String((unsigned char *)buf);
    
    Key[2].Short_Flag = 0;
  }

  /* ----- 双击处理 ----- */
  if (Key[0].Double_Flag == 1) {
    Tracking_Test_Flag = !Tracking_Test_Flag;
    if (Tracking_Test_Flag)
      Tuning_State = 1;
    Key[0].Double_Flag = 0;
  } else if (Key[1].Double_Flag == 1) // 启动第三问任务
  {
    extern volatile uint32_t ms_ticks;
    Task_Mode = 2;
    Task_2_State = 1;
    Key[1].Double_Flag = 0;
  } else if (Key[2].Double_Flag == 1) { // KEY3双击：减100微调
    extern int32_t PD42S1_Motor2_Pos;
    extern void BLE_send_String(unsigned char *str);
    extern uint8_t g_smd_target_uart;
    
    PD42S1_Motor2_Pos -= 100;
    g_smd_target_uart = 2;
    clear_uart_rx_buffer(2);
    smd_pos_mode(1, 0, 5, 80, PD42S1_Motor2_Pos);
    handle_ack(2, 50);
    
    char buf[32];
    sprintf(buf, "Motor2 Pos: %d\r\n", (int)PD42S1_Motor2_Pos);
    BLE_send_String((unsigned char *)buf);
    
    Key[2].Double_Flag = 0;
  }

  /* ----- 长按处理 ----- */
  if (Key[0].Long_Flag == 1) {
    extern uint8_t Test_Speed_Mode;
    Test_Speed_Mode = !Test_Speed_Mode; // 长按切换纯速度测试模式
    Key[0].Long_Flag = 0;
  } else if (Key[1].Long_Flag == 1) { // KEY2长按：启动第三问任务
    extern volatile uint32_t ms_ticks;
    Task_Mode = 2;
    Task_2_State = 1;
    count_10ms = 0;
    Key[1].Long_Flag = 0;
  } else if (Key[2].Long_Flag == 1) {
    extern volatile uint32_t ms_ticks;
    Task_Mode = 2;
    Task_2_State = 1;
    Key[2].Long_Flag = 0;
  }
}
