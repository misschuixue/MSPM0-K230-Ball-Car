#include "app_protocol.h"
#include "stdint.h"
#include "motor_ctrl.h"
#include "bsp_gyro.h"
#include "protocol.h"
#include "task.h"
#include "smd.h"
#include "usart.h"
#include "Encoder.h"
#include "gw_gray.h"
#include <stdio.h>

// ================= 外部变量声明 =================
extern uint8_t g_smd_target_uart;
extern Gyro_Struct *JY61P_Data;
extern float Stand_Hold_Yaw;
extern volatile uint32_t ms_ticks;

// K230 视觉模块下发的目标速度缓存
int16_t Gimbal_Speed_X = 0; 
int16_t Gimbal_Speed_Y = 0;
// 云台控制模式标志 (0: 位置模式/归位, 1: 视觉速度模式)
uint8_t K230_Ctrl_Mode = 0;

// K230 回传的小球实时坐标 (EE 帧)
volatile int32_t Ball_Pos_X = 0;

// K230 通过 FF 帧设置的小球目标停止位置 (基准位置)
// 任务二轨迹模式以此为基准计算 +5cm/-5cm, 任务三四定点模式直接使用
volatile float Ball_Target_Set_Pos = 295.0f;

// 小球目标位置 cm 偏移补偿 (用户可通过蓝牙 BO= 指令调整)
// 用于修正 K230 标定误差: 若"指定10cm停在5cm", 发送 BO=5 即可补偿
// M0 收到 FF 帧后: 实际目标 = K230目标 + Offset_CM * 22.4 (K230像素/cm系数)
volatile float Ball_Target_Offset_CM = 0.0f;

/**
  * @brief  K230 视觉数据帧解析状态机
  * @param  byte: 接收到的单字节数据
  */
void K230_Parse_Data(uint8_t byte)
{
    static uint8_t state = 0;
    static uint8_t cmd_type = 0; // 0: 速度模式(BB), 1: 绝对位置模式(DD), 2: 坐标反馈模式(EE), 3: 目标位置设置(FF)
    static uint8_t buf[8];       // 接收缓存
    static uint8_t data_idx = 0;

    switch(state) {
        case 0: if(byte == 0xAA) state = 1; else state = 0; break; 
        case 1: 
            if(byte == 0xBB) { cmd_type = 0; state = 2; data_idx = 0; } 
            else if(byte == 0xDD) { cmd_type = 1; state = 2; data_idx = 0; }
            else if(byte == 0xEE) { cmd_type = 2; state = 2; data_idx = 0; }
            else if(byte == 0xFF) { cmd_type = 3; state = 2; data_idx = 0; }
            else state = 0; 
            break; 
        case 2: 
            buf[data_idx++] = byte; 
            if((cmd_type == 0 && data_idx == 4) || (cmd_type == 1 && data_idx == 8) || (cmd_type == 2 && data_idx == 4) || (cmd_type == 3 && data_idx == 4)) state = 3;
            break;
        case 3: 
            if(byte == 0xCC) { // 校验帧尾
                if (cmd_type == 0) {
                    K230_Ctrl_Mode = 1; // 收到 BB 帧，切换为视觉速度模式
                    Gimbal_Speed_X = (int16_t)(buf[0] | (buf[1] << 8));
                    // 任务三(Ball_Task2_Mode)时，Y轴摆杆由本地PID接管，
                    // 禁止K230的BB帧覆盖Gimbal_Speed_Y，否则PID输出永远到不了电机
                    if (Car_Mode != Ball_Task2_Mode) {
                        Gimbal_Speed_Y = (int16_t)(buf[2] | (buf[3] << 8));
                    }
                } else if (cmd_type == 1) {
                    K230_Ctrl_Mode = 0; // 收到 DD 帧，切换为绝对位置模式
                    
                    int32_t pos_x = (int32_t)(buf[0] | (buf[1]<<8) | (buf[2]<<16) | (buf[3]<<24));
                    int32_t pos_y = (int32_t)(buf[4] | (buf[5]<<8) | (buf[6]<<16) | (buf[7]<<24));
                    
                    extern volatile int32_t K230_Target_Pos_X;
                    extern volatile int32_t K230_Target_Pos_Y;
                    extern volatile uint8_t K230_New_Pos_Flag;
                    
                    K230_Target_Pos_X = pos_x;
                    K230_Target_Pos_Y = pos_y;
                    K230_New_Pos_Flag = 1;
                } else if (cmd_type == 2) {
                    Ball_Pos_X = (int16_t)(buf[0] | (buf[1] << 8));
                } else if (cmd_type == 3) {
                    // FF帧: K230触摸屏调定的小球停止目标位置
                    int16_t target = (int16_t)(buf[0] | (buf[1] << 8));
                    // 应用 cm 偏移补偿: 实际目标 = K230目标 + 偏移cm * 22.4(像素/cm)
                    float compensated = target + Ball_Target_Offset_CM * 22.4f;
                    pid_Ball_Beam.Target = compensated;
                    Ball_Target_Set_Pos = compensated;  // 更新基准位置供任务二轨迹模式使用
                    extern uint8_t bt_user_override;
                    bt_user_override = 1;  // 用户已手动设置, 取消默认覆盖
                    // 蓝牙回传确认, 便于调试 K230->M0 通信链路
                    extern void BLE_send_String(unsigned char *str);
                    char ack_msg[48];
                    sprintf(ack_msg, "[FF] T:%d O:%.1fcm C:%d\r\n",
                            target, Ball_Target_Offset_CM, (int)compensated);
                    BLE_send_String((unsigned char *)ack_msg);
                }
            }
            state = 0; 
            break;
        default: state = 0; break;
    }
}
/*-------------------------------------------------------------------------------------------*/
/*--------------------------------上位机协议数据处理-----------------------------------------*/
/*-------------------------------------------------------------------------------------------*/

void Protocol_Datas_Proc(void)
{
	if(Car_Mode != Run_Mode)
	{		
		int32_t PROTOCOL_TEMP = 0;    //野火上位机上传实际值 (恢复为 int32_t)
		int32_t PROTOCOL_TARGET = 0;  //野火上位机上传目标值 (恢复为 int32_t)
		
		if(Car_Mode==Speed_Mode)//速度环模式
		{
			Turn_PID_Flag = 0;		//关闭循迹
			Angle_PID_Flag = 0;		//关闭角度环
			Gyro_PID_Flag = 0;		//关闭角速度环
			Distance_PID_Flag = 0;	//关闭距离环
			
			// 速度环通常不需要很细的小数，直接取整即可
			PROTOCOL_TARGET = (int32_t)Basic_Speed;
			if(Set_Motor_Param_Select==1) PROTOCOL_TEMP = (int32_t)Motor1_Speed;
			else if(Set_Motor_Param_Select==2) PROTOCOL_TEMP = (int32_t)Motor2_Speed;
		}
		else if(Car_Mode == Turn_Mode)//转向环模式
		{
			Turn_PID_Flag = 1;		
			Distance_PID_Flag = 0;
			Gyro_PID_Flag = 0;		
			Angle_PID_Flag = 0;	
			
			PROTOCOL_TARGET = 0; // 巡线目标误差通常为0
			PROTOCOL_TEMP = (int32_t)Huidu_Error;
		}
		else if(Car_Mode == Distance_Mode)//距离环模式
		{
			Turn_PID_Flag = 0;	
			Distance_PID_Flag = 1;		
			Gyro_PID_Flag = 0;		
			Angle_PID_Flag = 0;		
			
			// 距离环，放大100倍保留两位小数 (例如 123.45mm -> 发送 12345)
			PROTOCOL_TARGET = (int32_t)(Target_Distance * 100.0f);
			PROTOCOL_TEMP = (int32_t)(Measure_Distance * 100.0f);
		}   
		else if(Car_Mode == Gyro_Mode)//角速度环模式
		{
			Turn_PID_Flag = 0;		
			Distance_PID_Flag = 0;	
			Gyro_PID_Flag = 1;
			Angle_PID_Flag = 1;	
			
			// 角速度环，放大100倍保留两位小数
			PROTOCOL_TARGET = (int32_t)(Target_Gyro * 100.0f);
			if (JY61P_Data != NULL) {
			    PROTOCOL_TEMP = (int32_t)(JY61P_Data->gyro_z * 100.0f); 
			} else {
			    PROTOCOL_TEMP = 0;
			}
		}
		else if(Car_Mode == Angle_Mode)//角度环模式
		{
			Turn_PID_Flag = 0;		
			Distance_PID_Flag = 0;	
			Gyro_PID_Flag = 0;
			Angle_PID_Flag = 1;
			
			// 【核心修复】：角度环放大100倍，保留两位小数。
			// 上位机显示 1234 代表 12.34度！
			PROTOCOL_TARGET = (int32_t)(Target_Angle * 100.0f);
			if (JY61P_Data != NULL) {
			    PROTOCOL_TEMP = (int32_t)(JY61P_Data->total_z * 100.0f); 
			} else {
			    PROTOCOL_TEMP = 0;
			}
		}
		else if(Car_Mode == Stand_Mode)//驻车锁死模式
		{
			Turn_PID_Flag = 0;		
			Distance_PID_Flag = 0;	
			Gyro_PID_Flag = 0;
			Angle_PID_Flag = 0;
			
			// 驻车模式角度同样放大 100 倍
			PROTOCOL_TARGET = (int32_t)(Stand_Hold_Yaw * 100.0f); 
			PROTOCOL_TEMP = (int32_t)(JY61P_Data->total_z * 100.0f); 
		}
		
		// 20ms 发送间隔 (50Hz) - 提升波形图实时跟手感，降低视觉延迟
		static uint32_t last_send_time = 0;
		if (ms_ticks - last_send_time >= 20) {
			last_send_time = ms_ticks;
			
			if (Car_Mode != AutoTune_Mode) {
				if(Car_Mode == Speed_Mode) {
					// 速度模式下：仅发送左轮(CH1实际值)、右轮(CH2实际值)
					// 禁止回传目标值，避免覆盖上位机输入参数
					int32_t speed1 = (int32_t)Motor1_Speed; 
					int32_t speed2 = (int32_t)Motor2_Speed; 
					
					set_computer_value(SEND_FACT_CMD, CURVES_CH1, &speed1, 1);
					set_computer_value(SEND_FACT_CMD, CURVES_CH2, &speed2, 1);
				} else {
					set_computer_value(SEND_FACT_CMD, CURVES_CH1, &PROTOCOL_TEMP, 1);
				}
				
				// // 强制附加通道：仅在不冲突的模式下，在波形图 CH3 实时上报陀螺仪累计角度
				// if (Car_Mode != Stand_Mode && Car_Mode != Angle_Mode) {
				// 	extern Gyro_Struct *JY61P_Data;
				// 	int32_t gyro_z = (int32_t)JY61P_Data->total_z;
				// 	set_computer_value(SEND_FACT_CMD, CURVES_CH3, &gyro_z, 1);
				// }
			}
		}
	}
	else// 运行模式 (暂停数据上传以降低系统负载)
	{

	}
}


