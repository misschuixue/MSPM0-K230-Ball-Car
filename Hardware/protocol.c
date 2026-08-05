/***
 *******************************************************************************************************************************************************************
 * @file
 * @date
 * @author  野火
 * @brief   MSPM0G3507小车PID调试通用模板
 *************************************************************************************************************************************************************
 ***/
#include "protocol.h"
#include "motor_ctrl.h"
#include "usart.h"
#include <stdio.h>

struct prot_frame_parser_t {
  uint8_t *recv_ptr;
  uint16_t r_oft;
  uint16_t w_oft;
  uint16_t frame_len;
  uint16_t found_frame_head;
};
static struct prot_frame_parser_t parser;
static uint8_t recv_buf[PROT_FRAME_LEN_RECV];
/**
 * @brief 计算校验和
 * @param ptr：需要计算的数据
 * @param len：需要计算的长度
 * @retval 校验和
 */
uint8_t check_sum(uint8_t init, uint8_t *ptr, uint8_t len) {
  uint8_t sum = init;
  while (len--) {
    sum += *ptr;
    ptr++;
  }
  return sum;
}
/**
 * @brief   得到帧类型（帧命令）
 * @param   *frame:  数据帧
 * @param   head_oft: 帧头的偏移位置
 * @return  帧长度.
 */
static uint8_t get_frame_type(uint8_t *frame, uint16_t head_oft) {
  return (frame[(head_oft + CMD_INDEX_VAL) % PROT_FRAME_LEN_RECV] & 0xFF);
}
/**
 * @brief   得到帧类型（帧命令）
 * @param   *buf:  数据缓冲区.
 * @param   head_oft: 帧头的偏移位置
 * @return  帧长度.
 */
static uint16_t get_frame_len(uint8_t *frame, uint16_t head_oft) {
  return ((frame[(head_oft + LEN_INDEX_VAL + 0) % PROT_FRAME_LEN_RECV] << 0) |
          (frame[(head_oft + LEN_INDEX_VAL + 1) % PROT_FRAME_LEN_RECV] << 8) |
          (frame[(head_oft + LEN_INDEX_VAL + 2) % PROT_FRAME_LEN_RECV] << 16) |
          (frame[(head_oft + LEN_INDEX_VAL + 3) % PROT_FRAME_LEN_RECV]
           << 24)); // 合成帧长度
}
/**
 * @brief   获取 crc-16 校验值
 * @param   *frame:  数据缓冲区.
 * @param   head_oft: 帧头的偏移位置
 * @param   head_oft: 帧头的偏移位置
 * @return  帧长度.
 */
static uint8_t get_frame_checksum(uint8_t *frame, uint16_t head_oft,
                                  uint16_t frame_len) {
  return (frame[(head_oft + frame_len - 1) % PROT_FRAME_LEN_RECV]);
}
/**
 * @brief   查找帧头
 * @param   *buf:  数据缓冲区.
 * @param   ring_buf_len: 缓冲区大小
 * @param   start: 起始位置
 * @param   len: 需要查找的长度
 * @return  -1：没有找到帧头，其他值：帧头的位置.
 */
static int32_t recvbuf_find_header(uint8_t *buf, uint16_t ring_buf_len,
                                   uint16_t start, uint16_t len) {
  uint16_t i = 0;
  for (i = 0; i < (len - 3); i++) {
    if (((buf[(start + i + 0) % ring_buf_len] << 0) |
         (buf[(start + i + 1) % ring_buf_len] << 8) |
         (buf[(start + i + 2) % ring_buf_len] << 16) |
         (buf[(start + i + 3) % ring_buf_len] << 24)) == FRAME_HEADER) {
      return ((start + i) % ring_buf_len);
    }
  }
  return -1;
}
/**
 * @brief   计算为解析的数据长度   Undefined symbol taget_sp (referred from
 * protocol.o).
 * @param   *buf:  数据缓冲区.
 * @param   ring_buf_len: 缓冲区大小
 * @param   start: 起始位置
 * @param   end: 结束位置
 * @return  为解析的数据长度
 */
static int32_t recvbuf_get_len_to_parse(uint16_t frame_len,
                                        uint16_t ring_buf_len, uint16_t start,
                                        uint16_t end) {
  uint16_t unparsed_data_len = 0;
  if (start <= end)
    unparsed_data_len = end - start;
  else
    unparsed_data_len = ring_buf_len - start + end;

  if (frame_len > unparsed_data_len)
    return 0;
  else
    return unparsed_data_len;
}
/**
 * @brief   接收数据写入缓冲区
 * @param   *buf:  数据缓冲区.
 * @param   ring_buf_len: 缓冲区大小
 * @param   w_oft: 写偏移
 * @param   *data: 需要写入的数据
 * @param   *data_len: 需要写入数据的长度
 * @return  void.
 */
static void recvbuf_put_data(uint8_t *buf, uint16_t ring_buf_len,
                             uint16_t w_oft, uint8_t *data, uint16_t data_len) {
  if ((w_oft + data_len) > ring_buf_len) // 超过缓冲区尾
  {
    uint16_t data_len_part = ring_buf_len - w_oft; // 缓冲区剩余长度
    /* 数据分两段写入缓冲区*/
    memcpy(buf + w_oft, data, data_len_part); // 写入缓冲区尾
    memcpy(buf, data + data_len_part, data_len - data_len_part); // 写入缓冲区头
  } else
    memcpy(buf + w_oft, data, data_len); // 数据写入缓冲区
}
/**
 * @brief   查询帧类型（命令）
 * @param   *data:  帧数据
 * @param   data_len: 帧数据的大小
 * @return  帧类型（命令）.
 */
static uint8_t protocol_frame_parse(uint8_t *data, uint16_t *data_len) {
  uint8_t frame_type = CMD_NONE;
  uint16_t need_to_parse_len = 0;
  int16_t header_oft = -1;
  uint8_t checksum = 0;
  need_to_parse_len = recvbuf_get_len_to_parse(
      parser.frame_len, PROT_FRAME_LEN_RECV, parser.r_oft,
      parser.w_oft);         // 得到为解析的数据长度
  if (need_to_parse_len < 9) // 肯定还不能同时找到帧头和帧长度
    return frame_type;
  /* 还未找到帧头，需要进行查找*/
  if (0 == parser.found_frame_head) {
    /* 同步头为四字节，可能存在未解析的数据中最后一个字节刚好为同步头第一个字节的情况，
     */
    header_oft = recvbuf_find_header(parser.recv_ptr, PROT_FRAME_LEN_RECV,
                                     parser.r_oft, need_to_parse_len);
    if (0 <= header_oft) {
      /* 还未找到帧头，需要进行查找*/
      parser.found_frame_head = 1;
      parser.r_oft = header_oft;
      /* 还未找到帧头，需要进行查找*/
      if (recvbuf_get_len_to_parse(parser.frame_len, PROT_FRAME_LEN_RECV,
                                   parser.r_oft, parser.w_oft) < 9)
        return frame_type;
    } else {
      /* 已找到帧头*/
      parser.r_oft =
          ((parser.r_oft + need_to_parse_len - 3) % PROT_FRAME_LEN_RECV);
      return frame_type;
    }
  }
  /* 确认是否可以计算帧长*/
  if (0 == parser.frame_len) {
    parser.frame_len = get_frame_len(parser.recv_ptr, parser.r_oft);
    if (need_to_parse_len < parser.frame_len)
      return frame_type;
  }
  /* 未解析的数据中依然未找到帧头，丢掉此次解析过的所有数据*/
  if ((parser.frame_len + parser.r_oft - PROT_FRAME_LEN_CHECKSUM) >
      PROT_FRAME_LEN_RECV) {
    /* 计算帧长，并确定是否可以进行数据解析*/
    checksum = check_sum(checksum, parser.recv_ptr + parser.r_oft,
                         PROT_FRAME_LEN_RECV - parser.r_oft);
    checksum = check_sum(checksum, parser.recv_ptr,
                         parser.frame_len - PROT_FRAME_LEN_CHECKSUM +
                             parser.r_oft - PROT_FRAME_LEN_RECV);
  } else {
    /* 帧头位置确认，且未解析的数据超过帧长，可以计算校验和*/
    checksum = check_sum(checksum, parser.recv_ptr + parser.r_oft,
                         parser.frame_len - PROT_FRAME_LEN_CHECKSUM);
  }
  if (checksum ==
      get_frame_checksum(parser.recv_ptr, parser.r_oft, parser.frame_len)) {
    /* 数据帧被分为两部分，一部分在缓冲区尾，一部分在缓冲区头 */
    if ((parser.r_oft + parser.frame_len) > PROT_FRAME_LEN_RECV) {
      /* 数据帧可以一次性取完*/
      uint16_t data_len_part = PROT_FRAME_LEN_RECV - parser.r_oft;
      memcpy(data, parser.recv_ptr + parser.r_oft, data_len_part);
      memcpy(data + data_len_part, parser.recv_ptr,
             parser.frame_len - data_len_part);
    } else {
      /* 校验成功，拷贝整帧数据 */
      memcpy(data, parser.recv_ptr + parser.r_oft, parser.frame_len);
    }
    *data_len = parser.frame_len;
    frame_type = get_frame_type(parser.recv_ptr, parser.r_oft);
    /* 数据帧被分为两部分，一部分在缓冲区尾，一部分在缓冲区头*/
    parser.r_oft = (parser.r_oft + parser.frame_len) % PROT_FRAME_LEN_RECV;
  } else {
    /* 数据帧可以一次性取完*/
    parser.r_oft = (parser.r_oft + 1) % PROT_FRAME_LEN_RECV;
  }
  parser.frame_len = 0;
  parser.found_frame_head = 0;
  return frame_type;
}
/**
 * @brief   接收数据处理
 * @param   *data:  要计算的数据的数组.
 * @param   data_len: 数据的大小
 * @return  void.
 */
void protocol_data_recv(uint8_t *data, uint16_t data_len) {
  recvbuf_put_data(parser.recv_ptr, PROT_FRAME_LEN_RECV, parser.w_oft, data,
                   data_len); // 接收数据
  parser.w_oft = (parser.w_oft + data_len) % PROT_FRAME_LEN_RECV; // 计算写偏移
}
/**
 * @brief   接收数据处理
 * @param   void
 * @return  初始化结果.
 */
int32_t protocol_init(void) {
  memset(&parser, 0, sizeof(struct prot_frame_parser_t));
  /* 初始化分配数据接收与解析缓冲区*/
  parser.recv_ptr = recv_buf;
  return 0;
}
/**
 * @brief   初始化接收协议
 * @param   void
 * @return  -1：没有找到一个正确的命令.
 */
uint8_t Set_Motor_Param_Select =
    1; // 设置电机参数选择   1--一号电机  2--二号电机
int8_t receiving_process(void) {
  uint8_t frame_data[128];     // 要能放下最长的帧
  uint16_t frame_len = 0;      // 帧长度
  uint8_t cmd_type = CMD_NONE; // 命令类型
  while (1) {
    cmd_type =
        protocol_frame_parse(frame_data, &frame_len); // 读取命令类型
                                                      // 根据命令进行对应操作
    switch (cmd_type) {
      // 修改
    case CMD_NONE: {
      return -1;
    }
      // 设置PID的值
    case SET_P_I_D_CMD: {
      uint32_t temp0 = COMPOUND_32BIT(&frame_data[13]);
      float p_temp = *(float *)&temp0;
      uint32_t temp1 = COMPOUND_32BIT(&frame_data[17]);
      float i_temp = *(float *)&temp1;
      uint32_t temp2 = COMPOUND_32BIT(&frame_data[21]);
      float d_temp = *(float *)&temp2;

      char debug_msg[128];
      sprintf(debug_msg, "[DEBUG] SetPID: P=%.2f, I=%.2f, D=%.2f\r\n", p_temp,
              i_temp, d_temp);
      BLE_send_String((unsigned char *)debug_msg);

      if (Car_Mode == Speed_Mode) {
        // 不再区分 Set_Motor_Param_Select，直接同时修改左右两轮的 PID
        // 参数，保证双轮同步
        Set_PID_Param(&pid_Motor1_Speed, p_temp, i_temp, d_temp);
        Set_PID_Param(&pid_Motor2_Speed, p_temp, i_temp, d_temp);
      } else if (Car_Mode == Turn_Mode)
        Set_PID_Param(&pid_Turn, p_temp, i_temp, d_temp); // PID
      else if (Car_Mode == Distance_Mode)
        Set_PID_Param(&pid_Distance, p_temp, i_temp, d_temp); // PID
      else if (Car_Mode == Gyro_Mode)
        Set_PID_Param(&pid_Gyro, p_temp, i_temp, d_temp); // PID
      else if (Car_Mode == Angle_Mode)
        Set_PID_Param(&pid_Angle, p_temp, i_temp, d_temp); // PID
      else if (Car_Mode == Stand_Mode)
        Set_PID_Param(&pid_Stand_Angle, p_temp, i_temp, d_temp); // 接收弹簧刚度

    } break;
      // 目标值也就是PID控制的期望值
    case SET_TARGET_CMD: {
      int actual_temp = COMPOUND_32BIT(&frame_data[13]); // 得到目标值

      char debug_msg[128];
      sprintf(debug_msg, "[DEBUG] SetTarget: %d\r\n", actual_temp);
      BLE_send_String((unsigned char *)debug_msg);

      if (Car_Mode == Speed_Mode) {
        Basic_Speed = actual_temp; /* 设定基础线速度 */
        extern uint16_t Speed_Mode_Run_Timer;
        Speed_Mode_Run_Timer = 0;
        Motors_Enable();
      } else if (Car_Mode == Turn_Mode) {
        Basic_Speed = actual_temp; /* 设定转向差速目标 */
        Motors_Enable();
      } else if (Car_Mode == Distance_Mode) {
        /* 协议量纲转换：上位机精度 100 倍放大，此处还原为真实物理量 */
        Target_Distance = actual_temp / 100.0f;
        extern uint8_t Dynamic_Angle_Start_Flag;
        Dynamic_Angle_Start_Flag = 1; /* 触发轨迹规划状态机 */
        Motors_Enable();
      } else if (Car_Mode == Gyro_Mode) {
        Target_Gyro = actual_temp / 100.0f;
        extern uint8_t Dynamic_Angle_Start_Flag;
        Dynamic_Angle_Start_Flag = 1;
        Motors_Enable();
      } else if (Car_Mode == Angle_Mode) {
        /* 设定绝对航向角期望值 (Setpoints) */
        Target_Angle = actual_temp / 100.0f;
        extern uint8_t Dynamic_Angle_Start_Flag;
        Dynamic_Angle_Start_Flag = 1;
        Motors_Enable();
      }
    } break;
      // 接收到启动命令
    case START_CMD: {
      char debug_msg[128];
      sprintf(debug_msg,
              "[DEBUG] Start! M_Sel=%d, Basic=%.1f, P=%.2f, I=%.2f, D=%.2f\r\n",
              Set_Motor_Param_Select, Basic_Speed, pid_Motor1_Speed.Kp,
              pid_Motor1_Speed.Ki, pid_Motor1_Speed.Kd);
      BLE_send_String((unsigned char *)debug_msg);

      if (Car_Mode == Speed_Mode) {
        extern uint16_t Speed_Mode_Run_Timer;
        Speed_Mode_Run_Timer = 0; // 点击启动，也重置测试定时器！
        if (Set_Motor_Param_Select == 1)
          Motor1_Enable();
        else if (Set_Motor_Param_Select == 2)
          Motor2_Enable();
      } else {
        Motors_Enable();
      }

    } break;
      // 接收到停止命令
    case STOP_CMD: {
      if (Car_Mode == Speed_Mode) {
        if (Set_Motor_Param_Select == 1)
          Motor1_Disable();
        else if (Set_Motor_Param_Select == 2)
          Motor2_Disable();
      } else {
        Motors_Disable();
      }
    } break;
      // 收到单片机复位命令
    case RESET_CMD: {
      NVIC_SystemReset(); // 复位系统
    } break;
      // 收到设置电机参数命令
    case SET_PERIOD_CMD: {
      if (Car_Mode == Speed_Mode) {
        if (++Set_Motor_Param_Select >= 3)
          Set_Motor_Param_Select = 1;
      }

    } break;

    default:
      return -1;
    }
  }
}
/**********************************************************************************************/

/**
 * @brief 向上位机发送数据
 * @param cmd: 指令
 * @param ch: 通道
 * @param data: 数据指针
 * @param num: 数据长度
 * @retval 无
 */
void set_computer_value(uint8_t cmd, uint8_t ch, void *data, uint8_t num) {
  static packet_head_t set_packet;
  uint8_t sum = 0; // 校验和
  num *= 4;        // 一个数据4字节

  set_packet.head = FRAME_HEADER; // 帧头 0x59 48 5A 53
  set_packet.len = 0x0B + num;    // 长度
  set_packet.ch = ch;             // 通道
  set_packet.cmd = cmd;           // 指令

  sum = check_sum(0, (uint8_t *)&set_packet,
                  sizeof(set_packet));        // 计算包头校验和
  sum = check_sum(sum, (uint8_t *)data, num); // 计算数据校验和

  HAL_UART_Transmit((uint8_t *)&set_packet, sizeof(set_packet),
                    0xFFFFF);                               // 发送包头
  HAL_UART_Transmit((uint8_t *)data, num, 0xFFFFF);         // 发送数据
  HAL_UART_Transmit((uint8_t *)&sum, sizeof(sum), 0xFFFFF); // 发送校验和
}
