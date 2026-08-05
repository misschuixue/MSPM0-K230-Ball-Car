#include "hw_lcd.h"

// ==================== 提取的外部变量声明 ====================
extern volatile uint8_t ble_rx_idle_cnt;
extern uint8_t Tuning_Mode;
extern uint8_t Tuning_Cursor;

char tuner_msg1[32] = "Wait...";
char tuner_msg2[32] = "Score: N/A";
#include "app_lcd.h"
#include "bsp_hc05.h"
#include "protocol.h"
#include "stdbool.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

unsigned char ble_ring_buf[1024];
volatile uint16_t ble_ring_head = 0;
volatile uint16_t ble_ring_tail = 0;
volatile uint8_t ble_rx_idle_cnt = 0;

void BLE_Send_Bit(unsigned char ch) {
  uint32_t timeout = 500000;
  while ((!DL_UART_isTXFIFOEmpty(UART_1_INST)) && (timeout > 0)) {
    timeout--;
  }
  DL_UART_Main_transmitData(UART_1_INST, ch);
}

void BLE_send_String(unsigned char *str) {
  while (str && *str) {
    BLE_Send_Bit(*str++);
  }
}

void Bluetooth_Init(void) {
  NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
  NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

#include "motor_ctrl.h" // For pid_Turn etc.
#include <stdlib.h>     // For atof

volatile uint8_t telemetry_enabled = 0; // 0: OFF, 1: ON

void Receive_Bluetooth_Data(void) {
  // Used to remember loops
  static uint8_t v_loop = 0;  // 0:M1, 1:M2, 2:Turn, 3:Angle, 4:Dist
  static uint8_t v_param = 0; // 0:Kp, 1:Ki, 2:Kd

  static unsigned char local_buf[128];
  static uint8_t local_len = 0;

  // Timeout logic
  if (local_len > 0 && ble_rx_idle_cnt >= 20) {
    local_buf[local_len] = '\0';
    local_len = 0;
    ble_rx_idle_cnt = 0;
  }

  // Drain ring buffer
  while (ble_ring_tail != ble_ring_head) {
    ble_rx_idle_cnt = 0; // reset idle timeout
    char ch = ble_ring_buf[ble_ring_tail];
    ble_ring_tail = (ble_ring_tail + 1) % 1024;

    if (local_len < 127) {
      local_buf[local_len++] = ch;
    }

    if (ch == '\n' || ch == ']' || local_len >= 127) {
      local_buf[local_len] = '\0';

      for (int i = 0; i < local_len; i++) {
        if (local_buf[i] == '\r' || local_buf[i] == '\n') {
          local_buf[i] = '\0';
          break;
        }
      }

      // ================= 自动同步面板与调参对齐=================
      extern uint8_t OLED_View_Select;
      if (OLED_View_Select == 2) {
        v_loop = 2; // ?Turn
      }
      extern uint8_t Tuning_Loop;
      Tuning_Loop = v_loop;
      // =======================================================

      // 交互式PID调参解析
      float val = 0.0f;
      char msg[64];
      if (strncmp((char *)local_buf, "T1=", 3) == 0) {
        strncpy(tuner_msg1, (char *)&local_buf[3], 31);
        tuner_msg1[31] = '\0';
      } else if (strncmp((char *)local_buf, "T2=", 3) == 0) {
        strncpy(tuner_msg2, (char *)&local_buf[3], 31);
        tuner_msg2[31] = '\0';
      } else if (strncmp((char *)local_buf, "TP=", 3) == 0 || strncmp((char *)local_buf, "tp=", 3) == 0) {
        val = atof((char *)&local_buf[3]);
        pid_Turn.Kp = val;
        sprintf(msg, "Turn Kp set to: %d.%02d\r\n", (int)val, (int)(val * 100) % 100);
        BLE_send_String((unsigned char *)msg);
      } else if (strncmp((char *)local_buf, "TI=", 3) == 0 || strncmp((char *)local_buf, "ti=", 3) == 0) {
        val = atof((char *)&local_buf[3]);
        pid_Turn.Ki = val;
        sprintf(msg, "Turn Ki set to: %d.%02d\r\n", (int)val, (int)(val * 100) % 100);
        BLE_send_String((unsigned char *)msg);
      } else if (strncmp((char *)local_buf, "TD=", 3) == 0 || strncmp((char *)local_buf, "td=", 3) == 0) {
        val = atof((char *)&local_buf[3]);
        pid_Turn.Kd = val;
        sprintf(msg, "Turn Kd set to: %d.%02d\r\n", (int)val, (int)(val * 100) % 100);
        BLE_send_String((unsigned char *)msg);

        extern uint8_t Tuning_State;
        Tuning_State = 2; // AI HAS SENT NEW PARAMS! READY!
      }
      // 小球 PID 直通调参 (BP / BD)
      else if (strncmp((char *)local_buf, "BP=", 3) == 0 || strncmp((char *)local_buf, "bp=", 3) == 0) {
        val = atof((char *)&local_buf[3]);
        pid_Ball_Beam.Kp = val;
        sprintf(msg, "Ball Kp set to: %d.%02d\r\n", (int)val, (int)(val * 100) % 100);
        BLE_send_String((unsigned char *)msg);
      } else if (strncmp((char *)local_buf, "BD=", 3) == 0 || strncmp((char *)local_buf, "bd=", 3) == 0) {
        val = atof((char *)&local_buf[3]);
        pid_Ball_Beam.Kd = val;
        sprintf(msg, "Ball Kd set to: %d.%02d\r\n", (int)val, (int)(val * 100) % 100);
        BLE_send_String((unsigned char *)msg);
      }
      // 小球目标中心点 (BT) - 实时调整球停下的位置
      // 公式: Target = 320 + (期望cm - 2) * 25.28, 直接传 Target 原始像素值
      else if (strncmp((char *)local_buf, "BT=", 3) == 0 || strncmp((char *)local_buf, "bt=", 3) == 0) {
        val = atof((char *)&local_buf[3]);
        pid_Ball_Beam.Target = val;
        extern uint8_t bt_user_override;
        bt_user_override = 1;  // 用户已手动设置, 取消默认覆盖
        sprintf(msg, "Ball Target set to: %d\r\n", (int)val);
        BLE_send_String((unsigned char *)msg);
      }
      // 小球目标位置 cm 偏移补偿 (BO) - 修正 K230 标定误差
      // 例: K230指定10cm但小球停在5cm, 发送 BO=5 补偿5cm
      //     M0收到FF帧后: 实际目标 = K230目标 + BO值 * 22.4像素
      //     BO=0 关闭补偿 (默认)
      else if (strncmp((char *)local_buf, "BO=", 3) == 0 || strncmp((char *)local_buf, "bo=", 3) == 0) {
        val = atof((char *)&local_buf[3]);
        extern volatile float Ball_Target_Offset_CM;
        Ball_Target_Offset_CM = val;
        sprintf(msg, "Ball Offset: %d.%dcm\r\n", (int)val, abs((int)(val * 10) % 10));
        BLE_send_String((unsigned char *)msg);
      }
      // Target Speed (SP) - 专门用于悬空极速测试
      else if (strncmp((char *)local_buf, "SP=", 3) == 0 || strncmp((char *)local_buf, "sp=", 3) == 0) {
        extern float Target_Speed_Test;
        extern uint8_t Test_Speed_Mode;
        Target_Speed_Test = atof((char *)&local_buf[3]);
        if (Target_Speed_Test != 0.0f) {
          Test_Speed_Mode = 1;
          Motors_Enable();
        } else {
          Test_Speed_Mode = 0;
          Motors_Disable();
        }
        sprintf(msg, "Target Speed set to: %d\r\n", (int)Target_Speed_Test);
        BLE_send_String((unsigned char *)msg);
      } else if (strncmp((char *)local_buf, "MAX=", 4) == 0 || strncmp((char *)local_buf, "max=", 4) == 0) {
        extern uint8_t Max_Speed_Test_Flag;
        int flag = atoi((char *)&local_buf[4]);
        Max_Speed_Test_Flag = flag;
        sprintf(msg, "Max Speed Test Flag set to: %d\r\n", flag);
        BLE_send_String((unsigned char *)msg);
      }
      // Base Speed (BS) - 用于正常循迹调速，不触发测试模式，不自动发车
      else if (strncmp((char *)local_buf, "BS=", 3) == 0 || strncmp((char *)local_buf, "bs=", 3) == 0) {
        extern float Target_Speed_Test;
        Target_Speed_Test = atof((char *)&local_buf[3]);
        sprintf(msg, "Base Speed set to: %d\r\n", (int)Target_Speed_Test);
        BLE_send_String((unsigned char *)msg);
      }
      // Auto Tune Trigger (AT=1)
      else if (strncmp((char *)local_buf, "AT=1", 4) == 0 || strncmp((char *)local_buf, "at=1", 4) == 0) {
        extern uint8_t Car_Mode;
        extern void AutoTune_Init(void);
        Car_Mode = 3; // AutoTune Mode is 3 usually, or I can just use extern
        AutoTune_Init();
        Motors_Enable();
      }
      // 虚拟按键调参逻辑
      else if (strncmp((char *)local_buf, "[key,", 5) == 0) {
        char name[10] = {0};
        char action[10] = {0};
        if (sscanf((char *)local_buf, "[key,%9[^,],%9[^]]]", name, action) == 2) {
          if (strcmp(action, "down") == 0) {
            if (strcmp(name, "1") == 0) { 
              // 切换调节环(现在只有Turn)
              v_loop = 2;
              extern uint8_t Tuning_Loop;
              Tuning_Loop = v_loop;
              v_param = 0; // 切换环时默认回到 Kp
            } else if (strcmp(name, "2") == 0) {
              v_param = (v_param + 1) % 3;
            } else if (strcmp(name, "9") == 0) {
              telemetry_enabled = !telemetry_enabled;
              if (telemetry_enabled) {
                BLE_send_String((unsigned char *)"Plot: ON\r\n");
              } else {
                BLE_send_String((unsigned char *)"Plot: OFF\r\n");
              }
            } else if (strcmp(name, "3") == 0 || strcmp(name, "4") == 0 ||
                       strcmp(name, "5") == 0 || strcmp(name, "6") == 0 ||
                       strcmp(name, "7") == 0 || strcmp(name, "8") == 0) {
              float delta = 0;
              if (strcmp(name, "3") == 0) delta = 0.05f;
              else if (strcmp(name, "4") == 0) delta = -0.05f;
              else if (strcmp(name, "5") == 0) delta = 1.0f;
              else if (strcmp(name, "6") == 0) delta = -1.0f;
              else if (strcmp(name, "7") == 0) delta = 5.0f;
              else if (strcmp(name, "8") == 0) delta = -5.0f;

              pid_t *target_pid = NULL;
              if (v_loop == 0 || v_loop == 1) target_pid = &pid_Motor1_Speed;
              else if (v_loop == 2) target_pid = &pid_Turn;
              else if (v_loop == 3) target_pid = &pid_Angle;
              else if (v_loop == 4) target_pid = &pid_Distance;

              if (target_pid) {
                if (v_param == 0) {
                  target_pid->Kp += delta;
                  if (target_pid->Kp < 0.0f) target_pid->Kp = 0.0f;
                } else if (v_param == 1) {
                  target_pid->Ki += delta;
                  if (target_pid->Ki < 0.0f) target_pid->Ki = 0.0f;
                } else if (v_param == 2) {
                  target_pid->Kd += delta;
                  if (target_pid->Kd < 0.0f) target_pid->Kd = 0.0f;
                }
                
                // 【双轮同步补丁】：只要调节了速度环，强制让双轮参数一致
                if (target_pid == &pid_Motor1_Speed) {
                    pid_Motor2_Speed.Kp = pid_Motor1_Speed.Kp;
                    pid_Motor2_Speed.Ki = pid_Motor1_Speed.Ki;
                    pid_Motor2_Speed.Kd = pid_Motor1_Speed.Kd;
                }
              }
            }

            // 状态反馈
            const char *loop_names[] = {"M1", "M2", "Turn", "Angle", "Dist"};
            const char *param_names[] = {"Kp", "Ki", "Kd"};
            pid_t *curr_pid = NULL;
            if (v_loop == 0 || v_loop == 1) curr_pid = &pid_Motor1_Speed;
            else if (v_loop == 2) curr_pid = &pid_Turn;
            else if (v_loop == 3) curr_pid = &pid_Angle;
            else if (v_loop == 4) curr_pid = &pid_Distance;

            float curr_val = 0;
            if (curr_pid) {
                if (v_param == 0) curr_val = curr_pid->Kp;
                else if (v_param == 1) curr_val = curr_pid->Ki;
                else if (v_param == 2) curr_val = curr_pid->Kd;
            }

            extern volatile uint16_t telemetry_pause_ms;
            telemetry_pause_ms = 3000; // 暂停波形发送3 秒，以免刷屏

            sprintf(msg, "Sel: %s %s = %d.%02d\r\n", loop_names[v_loop],
                    param_names[v_param], (int)curr_val,
                    abs((int)(curr_val * 100) % 100));
            BLE_send_String((unsigned char *)msg);
          }
        }
      }
      // 解析滑杆指令，格式为 [slider,ID,VALUE]
      else if (strncmp((char *)local_buf, "[slider,", 8) == 0) {
        char slider_id[10] = {0};
        char slider_val_str[20] = {0};
        if (sscanf((char *)local_buf, "[slider,%9[^,],%19[^]]]", slider_id, slider_val_str) == 2) {
          float slider_val = atof(slider_val_str);

          // 添加专门针对小球PID的直通滑杆通道
          if (strcmp(slider_id, "BP") == 0 || strcmp(slider_id, "bp") == 0) {
              pid_Ball_Beam.Kp = slider_val;
          } else if (strcmp(slider_id, "BD") == 0 || strcmp(slider_id, "bd") == 0) {
              pid_Ball_Beam.Kd = slider_val;
          } else if (strcmp(slider_id, "T3S") == 0 || strcmp(slider_id, "t3s") == 0) {
              extern float Task3_Cruise_Speed;
              Task3_Cruise_Speed = slider_val;
          } else if (strcmp(slider_id, "T3A") == 0 || strcmp(slider_id, "t3a") == 0) {
              extern float Task3_Accel_Coeff;
              Task3_Accel_Coeff = slider_val;
          } else if (strcmp(slider_id, "T3D") == 0 || strcmp(slider_id, "t3d") == 0) {
              extern float Task3_Decel_Coeff;
              Task3_Decel_Coeff = slider_val;
          } else if (strcmp(slider_id, "T4S") == 0 || strcmp(slider_id, "t4s") == 0) {
              extern float Task4_Cruise_Speed;
              Task4_Cruise_Speed = slider_val;
          } else if (strcmp(slider_id, "T4A") == 0 || strcmp(slider_id, "t4a") == 0) {
              extern float Task4_Accel_Coeff;
              Task4_Accel_Coeff = slider_val;
          } else if (strcmp(slider_id, "T4D") == 0 || strcmp(slider_id, "t4d") == 0) {
              extern float Task4_Decel_Coeff;
              Task4_Decel_Coeff = slider_val;
          } else if (strcmp(slider_id, "T4B") == 0 || strcmp(slider_id, "t4b") == 0) {
              extern float Task4_Brake_Angle;
              Task4_Brake_Angle = slider_val;
          } else {
              pid_t *target_pid = NULL;
              if (v_loop == 0 || v_loop == 1) target_pid = &pid_Motor1_Speed;
              else if (v_loop == 2) target_pid = &pid_Turn;
              else if (v_loop == 3) target_pid = &pid_Angle;
              else if (v_loop == 4) target_pid = &pid_Distance;

          if (target_pid) {
            if (strcmp(slider_id, "1") == 0 || strstr(slider_id, "p") ||
                strstr(slider_id, "P") || strstr(slider_id, "1")) {
              target_pid->Kp = slider_val;
              v_param = 0; // 同步屏幕上的黄色高亮光标
              Tuning_Cursor = 0;
              Tuning_Mode = 1;
            } else if (strcmp(slider_id, "2") == 0 || strstr(slider_id, "i") ||
                       strstr(slider_id, "I") || strstr(slider_id, "2")) {
              target_pid->Ki = slider_val;
              v_param = 1;
              Tuning_Cursor = 1;
              Tuning_Mode = 1;
            } else if (strcmp(slider_id, "3") == 0 || strstr(slider_id, "d") ||
                       strstr(slider_id, "D") || strstr(slider_id, "3")) {
              target_pid->Kd = slider_val;
              v_param = 2;
              Tuning_Cursor = 2;
              Tuning_Mode = 1;
            } else {
              // 如果没有使用特定ID，则按原逻辑基于当前 v_param 修改
              if (v_param == 0) {
                target_pid->Kp = slider_val;
              } else if (v_param == 1) {
                target_pid->Ki = slider_val;
              } else if (v_param == 2) {
                target_pid->Kd = slider_val;
              }
            }
            
            // 【双轮同步补丁】
            if (target_pid == &pid_Motor1_Speed) {
                pid_Motor2_Speed.Kp = pid_Motor1_Speed.Kp;
                pid_Motor2_Speed.Ki = pid_Motor1_Speed.Ki;
                pid_Motor2_Speed.Kd = pid_Motor1_Speed.Kd;
            }
          }
          } // 闭合 else { ... }

          // 因为滑杆拖动时会产生海量高频数据，如果每次都回复会瞬间挤爆蓝牙发送通道
          // 所以在这里不回显Sel文本，只默默修改参数并重置暂停计时器
          extern volatile uint16_t telemetry_pause_ms;
          telemetry_pause_ms = 3000; // 暂停波形发送3 秒，以免刷屏
        }
      }
      local_len = 0;
      local_buf[0] = '\0';
    }
  }
}

void UART_1_INST_IRQHandler(void) {
  switch (DL_UART_getPendingInterrupt(UART_1_INST)) {
  case DL_UART_IIDX_RX:
  case DL_UART_IIDX_RX_TIMEOUT_ERROR: {
    while (!DL_UART_isRXFIFOEmpty(UART_1_INST)) {
      uint8_t ch = DL_UART_Main_receiveData(UART_1_INST);
      protocol_data_recv(&ch, 1); // 野火上位机协议接收 (Bluetooth)
      uint16_t next_head = (ble_ring_head + 1) % 1024;
      if (next_head != ble_ring_tail) {
        ble_ring_buf[ble_ring_head] = ch;
        ble_ring_head = next_head;
      }
    }
    break;
  }

  case DL_UART_IIDX_OVERRUN_ERROR:
  case DL_UART_IIDX_BREAK_ERROR:
  case DL_UART_IIDX_PARITY_ERROR:
  case DL_UART_IIDX_FRAMING_ERROR:
  case DL_UART_IIDX_NOISE_ERROR:
    // Clear all error flags to prevent the IRQ from looping infinitely and
    // freezing the system
    DL_UART_clearInterruptStatus(
        UART_1_INST,
        DL_UART_INTERRUPT_OVERRUN_ERROR | DL_UART_INTERRUPT_BREAK_ERROR |
            DL_UART_INTERRUPT_PARITY_ERROR | DL_UART_INTERRUPT_FRAMING_ERROR |
            DL_UART_INTERRUPT_NOISE_ERROR);
    {
      volatile uint8_t dump = DL_UART_Main_receiveData(UART_1_INST);
      (void)dump;
    }
    while (!DL_UART_isRXFIFOEmpty(UART_1_INST)) {
      DL_UART_Main_receiveData(UART_1_INST); // flush broken data
    }
    break;

  default:
    DL_UART_clearInterruptStatus(UART_1_INST, 0xFFFFFFFF);
    break;
  }
}
