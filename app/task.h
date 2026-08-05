/**
 * @file task.h
 * @brief 底层驱动与业务逻辑模块
 * @details ���ļ��� TeamSpark ??2026 NUEDC �����ڼ��ع�������ͳһ����淶??
 * @author TeamSpark
 * @date 2026-07-10
 */
#ifndef __TASK_H__
#define __TASK_H__

#include "main.h"

void Task_1(void);
extern uint8_t Task_Mode;
extern float Task1_Time_Sec; // 任务一用时
extern float Task2_Time_Sec; // 任务二用时
extern uint8_t Task1_Time_flag;
extern uint8_t Task2_Time_flag;
extern uint8_t Task1_run_flag;
extern uint8_t Task2_run_flag;
void Task_Scheduler(void);

extern uint8_t handle_ack(uint8_t motor_id, uint32_t over_time);
extern void clear_uart_rx_buffer(uint8_t motor_id);

extern uint8_t Task_1_State;  // 任务一状态机
extern float Start_Yaw;

extern uint8_t Task_Select;
extern uint8_t flag_state;
extern uint8_t flag_cnt;
extern uint8_t car_num;
extern uint8_t flag_warn;
extern uint8_t Target_Angle_flag;

void Task_2(void);
extern uint8_t Task_2_State;

void Task_3(void);
extern uint8_t Task_3_State;
extern uint8_t Task3_Time_flag;
extern float Task3_Time_Sec;
extern float Task3_Cruise_Speed;
extern float Task3_Accel_Coeff;
extern float Task3_Decel_Coeff;
extern float Task3_Start_Speed;

void Task_4(void);
extern uint8_t Task_4_State;
extern uint8_t Task4_Time_flag;
extern float Task4_Time_Sec;
extern float Task4_Cruise_Speed;
extern float Task4_Accel_Coeff;
extern float Task4_Decel_Coeff;
extern float Task4_Start_Speed;
extern float Task4_Brake_Angle; // 角度触发刹车阈值(默认347-90=257)

#endif
