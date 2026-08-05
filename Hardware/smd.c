#include "smd.h"
#include "string.h"
#include "usart.h"

/* 添加联合体 便于存储浮点数 */
union {
  float f;
  uint8_t b[4];
} data_u;

// 定义路由标志，默认为1
uint8_t g_smd_target_uart = 1;
volatile uint8_t smd_uart_busy = 0;

void smd_send_data(uint8_t *data, uint8_t len) {
  if (smd_uart_busy) return;
  smd_uart_busy = 1;
  uint8_t target_uart = g_smd_target_uart;

  for (uint8_t i = 0; i < len; i++) {
    uint32_t timeout = 500000;
    // 路由逻辑：根据变量决定从哪个串口发出去
    if (target_uart == 1) {
      while ((!DL_UART_isTXFIFOEmpty(UART_MOTOR_INST)) && (timeout > 0)) {
        timeout--;
      }
      DL_UART_Main_transmitData(UART_MOTOR_INST, data[i]);
    } else if (target_uart == 2) {
      while ((!DL_UART_isTXFIFOEmpty(UART_MOTOR_2_INST)) && (timeout > 0)) {
        timeout--;
      }
      DL_UART_Main_transmitData(UART_MOTOR_2_INST, data[i]);
    }
  }
  smd_uart_busy = 0;
}
/**
 * @brief   计算校验和函数
 * @param   data: 数据缓冲区
 * @param   len:  长度
 * @retval  校验和
 */
uint8_t smd_checksum(const uint8_t *data, uint8_t length) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum;
}

/**
 * @brief   校准编码器
 * @param    addr     : 电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 命令状态 + 校验字节 + 帧尾
 */
void smd_cal_encoder(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_CAL_ENCODER;      /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    复位重启
 * @param    addr    : 电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 命令状态 + 校验字节 + 帧尾
 */
void smd_restart(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_RESTART;          /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    恢复出厂设置
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 命令状态 + 校验字节 + 帧尾
 */
void smd_reset_factory(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_RESET_FACTORY;    /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_param_save(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_PARAM_SAVE;       /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取软硬件版本信息
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_soft_hard_ver(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;             /* 帧头 */
  cmd[1] = addr;                   /* 地址 */
  cmd[2] = FCT_READ_SOFT_HARD_VER; /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3);   /* 校验和 */
  cmd[4] = FRAME_TAIL;             /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取相电阻和相电感
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_psi(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_PSI;         /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取相电阻和相电感
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_phase_res_ind(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;             /* 帧头 */
  cmd[1] = addr;                   /* 地址 */
  cmd[2] = FCT_READ_PHASE_RES_IND; /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3);   /* 校验和 */
  cmd[4] = FRAME_TAIL;             /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取相电流
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_phase_ma(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_PHASE_MA;    /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取总线电压
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_vol(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_VOL;         /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取电流环PID参数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_ma_pid(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_MA_PID;      /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取速度环PID参数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_speed_pid(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_SPEED_PID;   /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取位置环PID参数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_pos_pid(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_POS_PID;     /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取输入累计脉冲数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_tatal_pulse(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_TOTAL_PULSE; /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读电机实时转速
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_rotate_speed(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;            /* 帧头 */
  cmd[1] = addr;                  /* 地址 */
  cmd[2] = FCT_READ_ROTATE_SPEED; /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3);  /* 校验和 */
  cmd[4] = FRAME_TAIL;            /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取电机实时位置
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_pos(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_POS;         /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取电机位置误差
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_pos_error(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_POS_ERROR;   /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取电机运行状态
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_motor_sta(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_MOTOR_STA;   /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取堵转标志
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_clog_flag(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_CLOG_FLAG;   /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取堵转电流
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_clog_current(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_CLOG_CUR;    /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读使能状态
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_enable_sta(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_ENABLE_STA;  /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读到位状态
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_arrived_sta(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_ARRIVED_STA; /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取系统参数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_sys_params(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_READ_SYS_PARAM;   /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取驱动参数
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_read_drive_params(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;            /* 帧头 */
  cmd[1] = addr;                  /* 地址 */
  cmd[2] = FCT_READ_DRIVE_PARAMS; /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3);  /* 校验和 */
  cmd[4] = FRAME_TAIL;            /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    设置从机地址
 * @param    addr     :  电机地址
 * @param    new_addr :  要设置的新地址
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_slave_add(uint8_t addr, uint8_t new_addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_SET_SLAVE_ADD;    /* 功能码 */
  cmd[3] = new_addr;             /* 电机新的地址 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置分组地址
 * @param    addr     :  电机地址
 * @param    new_addr :  要设置的新地址
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_group_add(uint8_t addr, uint8_t new_addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_SET_GROUP_ADD;    /* 功能码 */
  cmd[3] = new_addr;             /* 电机新的地址 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置工作模式
 * @param    addr   :   电机地址
 * @param    mode   :
 工作模式1~7分别对应：通信位置模式、通信速度模式、通信力矩模式、
                                    脉宽位置模式、脉宽速度模式、脉宽力矩模式、脉冲模式
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_mode(uint8_t addr, uint8_t mode) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_SET_MODE;         /* 功能码 */
  cmd[3] = mode;                 /* 工作模式 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置位置环PID参数
 * @param    addr   :   电机地址
 * @param
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_pos_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd) {
  uint8_t cmd[32] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;      /* 帧头 */
  cmd[1] = addr;            /* 地址 */
  cmd[2] = FCT_SET_POS_PID; /* 功能码 */
  /* PID参数 */
  cmd[3] = (uint8_t)((kp >> 24) & 0xFF);
  cmd[4] = (uint8_t)((kp >> 16) & 0xFF);
  cmd[5] = (uint8_t)((kp >> 8) & 0xFF);
  cmd[6] = (uint8_t)((kp >> 0) & 0xFF);

  cmd[7] = (uint8_t)((ki >> 24) & 0xFF);
  cmd[8] = (uint8_t)((ki >> 16) & 0xFF);
  cmd[9] = (uint8_t)((ki >> 8) & 0xFF);
  cmd[10] = (uint8_t)((ki >> 0) & 0xFF);

  cmd[11] = (uint8_t)((kd >> 24) & 0xFF);
  cmd[12] = (uint8_t)((kd >> 16) & 0xFF);
  cmd[13] = (uint8_t)((kd >> 8) & 0xFF);
  cmd[14] = (uint8_t)((kd >> 0) & 0xFF);

  cmd[15] = smd_checksum(cmd, 15); /* 校验和 */
  cmd[16] = FRAME_TAIL;            /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 17);
}

/**
 * @brief    设置位置环力矩限制
 * @param    addr   :   电机地址
 * @param    torque :   位置环力矩限制（100~3000mA）
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_pos_torque(uint8_t addr, int16_t torque) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;         /* 帧头 */
  cmd[1] = addr;               /* 地址 */
  cmd[2] = FCT_SET_POS_TORQUE; /* 功能码 */
  /* 力矩参数 */
  cmd[3] = (uint8_t)((torque >> 8) & 0xFF);
  cmd[4] = (uint8_t)((torque >> 0) & 0xFF);
  cmd[5] = smd_checksum(cmd, 5); /* 校验和 */
  cmd[6] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 7);
}

/**
 * @brief    设置细分
 * @param    addr   :  电机地址
 * @param    step   :  细分(取值范围：1~256)
 * @retval   从机应答 : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_step(uint8_t addr, uint16_t step) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;                    /* 帧头 */
  cmd[1] = addr;                          /* 地址 */
  cmd[2] = FCT_SET_STEP;                  /* 功能码 */
  cmd[3] = (uint8_t)((step >> 8) & 0xFF); /* 细分 */
  cmd[4] = (uint8_t)((step >> 0) & 0xFF); /* 细分 */
  cmd[5] = smd_checksum(cmd, 5);          /* 校验和 */
  cmd[6] = FRAME_TAIL;                    /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 7);
}

/**
 * @brief    设置目标电流（仅力矩模式下有效）
 * @param    addr   :  电机地址
 * @param    ma     :  电流0~3000mA
 * @retval   从机应答 : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_ma(uint8_t addr, int16_t ma) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD; /* 帧头 */
  cmd[1] = addr;       /* 地址 */
  cmd[2] = FCT_SET_MA; /* 功能码 */
  /* 电流值mA */
  cmd[3] = (uint8_t)((ma >> 8) & 0xFF);
  cmd[4] = (uint8_t)((ma >> 0) & 0xFF);

  cmd[5] = smd_checksum(cmd, 5); /* 校验和 */
  cmd[6] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 7);
}

/**
 * @brief    设置串口波特率
 * @param    addr    :  电机地址
 * @param    baud    :  波特率
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_uart_baud(uint8_t addr, uint32_t baud) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;        /* 帧头 */
  cmd[1] = addr;              /* 地址 */
  cmd[2] = FCT_SET_UART_BAUD; /* 功能码 */
  /* 波特率 */
  cmd[3] = (uint8_t)((baud >> 24) & 0xFF);
  cmd[4] = (uint8_t)((baud >> 16) & 0xFF);
  cmd[5] = (uint8_t)((baud >> 8) & 0xFF);
  cmd[6] = (uint8_t)((baud >> 0) & 0xFF);
  cmd[7] = smd_checksum(cmd, 7); /* 校验和 */
  cmd[8] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 9);
}

/**
 * @brief    设置CAN波特率
 * @param    addr    :  电机地址
 * @param    baud    :  波特率（单位Kbps）
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_can_baud(uint8_t addr, uint16_t baud) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;       /* 帧头 */
  cmd[1] = addr;             /* 地址 */
  cmd[2] = FCT_SET_CAN_BAUD; /* 功能码 */
  /* 波特率 */
  cmd[3] = (uint8_t)((baud >> 8) & 0xFF);
  cmd[4] = (uint8_t)((baud >> 0) & 0xFF);
  cmd[5] = smd_checksum(cmd, 5); /* 校验和 */
  cmd[6] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 7);
}

/**
 * @brief    设置MODBUS
 * @param    addr      :  电机地址
 * @param    modbus    :  是否使用MODBUS协议
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_modbus(uint8_t addr, uint8_t modbus) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_SET_MODBUS;       /* 功能码 */
  cmd[3] = modbus;               /* 填充字节 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置堵转保护
 * @param    addr    :  电机地址
 * @param    en      :  1开启堵转保护 0关闭堵转保护
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_clog_pro(uint8_t addr, uint8_t en) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_SET_CLOG_PRO;     /* 功能码 */
  cmd[3] = en;                   /* 堵转保护标志 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置堵转电流
 * @param    addr    :  电机地址
 * @param    ma      :  0~3000mA
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_clog_current(uint8_t addr, int16_t ma) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;                  /* 帧头 */
  cmd[1] = addr;                        /* 地址 */
  cmd[2] = FCT_SET_CLOG_CUR;            /* 功能码 */
  cmd[3] = (uint8_t)((ma >> 8) & 0xFF); /* 低字节 */
  cmd[4] = (uint8_t)((ma >> 0) & 0xFF); /* 高字节 */
  cmd[5] = smd_checksum(cmd, 5);        /* 校验和 */
  cmd[6] = FRAME_TAIL;                  /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 7);
}

/**
 * @brief    设置CAN_ID
 * @param    addr     :  电机地址
 * @param    id       :  29位扩展帧ID
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_can_id(uint8_t addr, uint32_t id) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;                   /* 帧头 */
  cmd[1] = addr;                         /* 地址 */
  cmd[2] = FCT_SET_CAN_ID;               /* 功能码 */
  cmd[3] = (uint8_t)((id >> 24) & 0xFF); /* 低字节 */
  cmd[4] = (uint8_t)((id >> 16) & 0xFF); /* 高字节 */
  cmd[5] = (uint8_t)((id >> 8) & 0xFF);
  cmd[6] = (uint8_t)((id >> 0) & 0xFF);
  cmd[7] = smd_checksum(cmd, 7); /* 校验和 */
  cmd[8] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 9);
}

/**
 * @brief    设置DIR正转电平
 * @param    addr   :  电机地址
 * @param    dir    :  方向  0 高电平正转  1 低电平反转
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_dir_level(uint8_t addr, uint8_t dir) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_SET_DIR_LEVEL;    /* 功能码 */
  cmd[3] = dir;                  /* 旋转方向 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置EN脚有效电平
 * @param    addr   :  电机地址
 * @param    en     :  0：低电平有效， 1：高电平有效， 2：保持有效
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_en_level(uint8_t addr, uint8_t en) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_SET_EN_LEVEL;     /* 功能码 */
  cmd[3] = en;                   /* EN */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置指令是否回响
 * @param    addr   :  电机地址
 * @param    echo   :  0：回响 ， 1：不回响
 * @retval   从机应答：帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_cmd_echo(uint8_t addr, uint8_t echo) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_SET_CMD_ECHO;     /* 功能码 */
  cmd[3] = echo;                 /* 回响标志 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置按键锁定
 * @param    addr     :  电机地址
 * @param    lock     :  1 上锁 0 解锁
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_key_lock(uint8_t addr, uint8_t lock) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_SET_KEY_LOCK;     /* 功能码 */
  cmd[3] = lock;                 /* 上锁标志 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置自动熄屏
 * @param    addr    :  电机地址
 * @param    en      :  1 开启自动熄屏， 0 关闭自动熄屏，
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_auto_not_display(uint8_t addr, uint8_t en) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;               /* 帧头 */
  cmd[1] = addr;                     /* 地址 */
  cmd[2] = FCT_SET_AUTO_NOT_DISPLAY; /* 功能码 */
  cmd[3] = en;                       /* 自动熄屏标志 */
  cmd[4] = smd_checksum(cmd, 4);     /* 校验和 */
  cmd[5] = FRAME_TAIL;               /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief   设置IO启动电平
 * @param    addr    :  电机地址
 * @param    level   :  0 低电平启动 1 高电平启动
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_io_start_level(uint8_t addr, uint8_t level) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;             /* 帧头 */
  cmd[1] = addr;                   /* 地址 */
  cmd[2] = FCT_SET_IO_START_LEVEL; /* 功能码 */
  cmd[3] = level;                  /* 自动熄屏标志 */
  cmd[4] = smd_checksum(cmd, 4);   /* 校验和 */
  cmd[5] = FRAME_TAIL;             /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置速度环PID参数
 * @param    addr   :   电机地址
 * @param
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_set_speed_pid(uint8_t addr, uint32_t kp, uint32_t ki, uint32_t kd) {
  uint8_t cmd[32] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;        /* 帧头 */
  cmd[1] = addr;              /* 地址 */
  cmd[2] = FCT_SET_SPEED_PID; /* 功能码 */
  /* PID参数 */
  cmd[3] = (uint8_t)((kp >> 24) & 0xFF);
  cmd[4] = (uint8_t)((kp >> 16) & 0xFF);
  cmd[5] = (uint8_t)((kp >> 8) & 0xFF);
  cmd[6] = (uint8_t)((kp >> 0) & 0xFF);

  cmd[7] = (uint8_t)((ki >> 24) & 0xFF);
  cmd[8] = (uint8_t)((ki >> 16) & 0xFF);
  cmd[9] = (uint8_t)((ki >> 8) & 0xFF);
  cmd[10] = (uint8_t)((ki >> 0) & 0xFF);

  cmd[11] = (uint8_t)((kd >> 24) & 0xFF);
  cmd[12] = (uint8_t)((kd >> 16) & 0xFF);
  cmd[13] = (uint8_t)((kd >> 8) & 0xFF);
  cmd[14] = (uint8_t)((kd >> 0) & 0xFF);

  cmd[15] = smd_checksum(cmd, 15); /* 校验和 */
  cmd[16] = FRAME_TAIL;            /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 17);
}

/**
 * @brief    设置左限位原点位置
 * @param    addr     :  电机地址
 * @param    pos      :  零点位置
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_set_left_pos(uint8_t addr, int32_t pos) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;                    /* 帧头 */
  cmd[1] = addr;                          /* 地址 */
  cmd[2] = FCT_ORIGIN_SET_LEFT_POS;       /* 功能码 */
  cmd[3] = (uint8_t)((pos >> 24) & 0xFF); /* 零点位置 */
  cmd[4] = (uint8_t)((pos >> 16) & 0xFF);
  cmd[5] = (uint8_t)((pos >> 8) & 0xFF);
  cmd[6] = (uint8_t)((pos >> 0) & 0xFF);
  cmd[7] = smd_checksum(cmd, 7); /* 校验和 */
  cmd[8] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 9);
}

/**
 * @brief    找零点（有限位回零或无限位回零）
 * @param    addr              :  电机地址
 * @param    limit_enable      :  0 无限位  1 有限位
 * @param    dir               :  0 正转    1 反转
 * @param    speed_rpm         :  转速RPM(600RPM以内 建议100RPM)
 * @param    curr_limit        :
 * 无限位判断堵死电流mA(仅无限位回零有效建议200~1000)
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_homing_by_limit(uint8_t addr, uint8_t limit_enable, uint8_t dir,
                                int32_t speed_rpm, int16_t curr_limit) {
  uint8_t cmd[32] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;            /* 帧头 */
  cmd[1] = addr;                  /* 地址 */
  cmd[2] = FCT_ORIGIN_LIMIT_HOME; /* 功能码 */
  cmd[3] = limit_enable;          /* 有无限位 */
  cmd[4] = dir;                   /* 方向 */
  cmd[5] = (uint8_t)((speed_rpm >> 24) & 0xFF);
  cmd[6] = (uint8_t)((speed_rpm >> 16) & 0xFF);
  cmd[7] = (uint8_t)((speed_rpm >> 8) & 0xFF);
  cmd[8] = (uint8_t)((speed_rpm >> 0) & 0xFF);

  cmd[9] = (uint8_t)((curr_limit >> 8) & 0xFF);
  cmd[10] = (uint8_t)((curr_limit >> 0) & 0xFF);

  cmd[11] = smd_checksum(cmd, 11); /* 校验和 */
  cmd[12] = FRAME_TAIL;            /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 13);
}

/**
 * @brief    触发回零
 * @param    addr     :  电机地址
 * @param    mode     :  0 单圈回零  1 就近回零  2 多圈回零
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_trig(uint8_t addr, uint8_t mode) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_ORIGIN_TRIG;      /* 功能码 */
  cmd[3] = mode;                 /* 回零方式 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    强制中断并退出回零操作
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_break(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_ORIGIN_BREAK;     /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    读取原点回零参数
 * @param    addr    :  电机地址
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_read_params(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;             /* 帧头 */
  cmd[1] = addr;                   /* 地址 */
  cmd[2] = FCT_ORIGIN_READ_PARAMS; /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3);   /* 校验和 */
  cmd[4] = FRAME_TAIL;             /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    修改原点回零超时时间
 * @param    addr      :  电机地址
 * @param    timout    :  找零点超时时间（单位ms）
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_set_params(uint8_t addr, uint32_t timout) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;            /* 帧头 */
  cmd[1] = addr;                  /* 地址 */
  cmd[2] = FCT_ORIGIN_SET_PARAMS; /* 功能码 */
  cmd[3] = (uint8_t)((timout >> 24) & 0xFF);
  cmd[4] = (uint8_t)((timout >> 16) & 0xFF);
  cmd[5] = (uint8_t)((timout >> 8) & 0xFF);
  cmd[6] = (uint8_t)((timout >> 0) & 0xFF);
  cmd[7] = smd_checksum(cmd, 7); /* 校验和 */
  cmd[8] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 9);
}

/**
 * @brief    读取回零状态
 * @param    addr      :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_read_sta(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_ORIGIN_READ_STA;  /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    设置上电是否需自动回零
 * @param    addr      :  电机地址
 * @param    flag      :  0 不自动回零 1 自动回零
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_aoto_zero(uint8_t addr, uint8_t flag) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_ORIGIN_AOTO_ZERO; /* 功能码 */
  cmd[3] = flag;                 /* 设置是否需自动回零 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    设置右限位原点位置
 * @param    addr     :  电机地址
 * @param    pos      :  零点位置
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_set_right_pos(uint8_t addr, int32_t pos) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;                    /* 帧头 */
  cmd[1] = addr;                          /* 地址 */
  cmd[2] = FCT_ORIGIN_SET_RIGHT_POS;      /* 功能码 */
  cmd[3] = (uint8_t)((pos >> 24) & 0xFF); /* 零点位置 */
  cmd[4] = (uint8_t)((pos >> 16) & 0xFF);
  cmd[5] = (uint8_t)((pos >> 8) & 0xFF);
  cmd[6] = (uint8_t)((pos >> 0) & 0xFF);
  cmd[7] = smd_checksum(cmd, 7); /* 校验和 */
  cmd[8] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 9);
}

/**
 * @brief   设置左右限位开关状态（开启后电机运动范围受限左右限位原点位置）
 * @param    addr    :  电机地址
 * @param    level   :  0 关闭左右限位 1 开启左右限位
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_origin_l_r_switch(uint8_t addr, uint8_t ctrl) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_ORIGIN_SWITCH;    /* 功能码 */
  cmd[3] = ctrl;                 /* 左右限位开关状态 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    力矩模式
 * @param    addr      :  电机地址
 * @param    dir       :  方向       0为CW(顺时针)，其余为CCW(逆时针)
 * @param    current   :  电流       ，范围0 - 3000mA
 * @retval   从机应答   :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_torque_mode(uint8_t addr, uint8_t dir, uint16_t current) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;                       /* 帧头 */
  cmd[1] = addr;                             /* 地址 */
  cmd[2] = FCT_TORQUE_MODE;                  /* 功能码 */
  cmd[3] = dir;                              /* 方向 */
  cmd[4] = (uint8_t)((current >> 8) & 0xFF); /* 电流高8位字节 */
  cmd[5] = (uint8_t)((current >> 0) & 0xFF); /* 电流低8位字节 */
  cmd[6] = smd_checksum(cmd, 6);             /* 校验和 */
  cmd[7] = FRAME_TAIL;                       /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 8);
}

/**
 * @brief    速度模式
 * @param    addr      :  电机地址
 * @param    dir       :  方向       ，0正转，1反转
 * @param    acc       :  加速度     ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed     :  速度       ，范围0.1 - 3000RPM
 * @retval   从机应答   :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_speed_mode(uint8_t addr, uint8_t dir, uint8_t acc, float speed) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;     /* 帧头 */
  cmd[1] = addr;           /* 地址 */
  cmd[2] = FCT_SPEED_MODE; /* 功能码 */
  cmd[3] = dir;            /* 方向 */
  cmd[4] = acc;            /* 加速度，注意：0是直接启动 */
  data_u.f = speed;        /* 速度(RPM) */
  cmd[5] = data_u.b[3];
  cmd[6] = data_u.b[2];
  cmd[7] = data_u.b[1];
  cmd[8] = data_u.b[0];
  cmd[9] = smd_checksum(cmd, 9); /* 校验和 */
  cmd[10] = FRAME_TAIL;          /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 11);
}

/**
 * @brief    绝对位置模式控制
 * @param    addr     :  电机地址
 * @param    dir      :  方向        0为CW(顺时针)，其余为CCW(逆时针)
 * @param    acc      :  加速度      ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed    :  最大速度    ，范围0 - 3000RPM
 * @param    pulses   :  脉冲数      ，范围0- (2^32 - 1)个
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pos_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed,
                  uint32_t pulses) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;   /* 帧头 */
  cmd[1] = addr;         /* 地址 */
  cmd[2] = FCT_POS_MODE; /* 功能码 */
  cmd[3] = dir;          /* 方向 */
  cmd[4] = acc;          /* 加速度，注意：0是直接启动 */
  cmd[5] = (uint8_t)((speed >> 8) & 0xFF);   /* 速度(RPM)高8位字节 */
  cmd[6] = (uint8_t)((speed >> 0) & 0xFF);   /* 速度(RPM)低8位字节 */
  cmd[7] = (uint8_t)((pulses >> 24) & 0xFF); /* 脉冲数(bit24 - bit31 ) */
  cmd[8] = (uint8_t)((pulses >> 16) & 0xFF); /* 脉冲数(bit16 - bit23) */
  cmd[9] = (uint8_t)((pulses >> 8) & 0xFF);  /* 脉冲数(bit8  - bit15) */
  cmd[10] = (uint8_t)((pulses >> 0) & 0xFF); /* 脉冲数(bit0  - bit7) */
  cmd[11] = smd_checksum(cmd, 11);           /* 校验和 */
  cmd[12] = FRAME_TAIL;                      /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 13);
}

/**
 * @brief    相对位置模式控制
 * @param    addr     :  电机地址
 * @param    dir      :  方向        0为CW(顺时针)，其余为CCW(逆时针)
 * @param    acc      :  加速度      ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed    :  最大速度    ，范围0 - 3000RPM
 * @param    pulses   :  脉冲数      ，范围0- (2^32 - 1)个
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pos_rel_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed,
                      uint32_t pulses) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;       /* 帧头 */
  cmd[1] = addr;             /* 地址 */
  cmd[2] = FCT_POS_REL_MODE; /* 功能码 */
  cmd[3] = dir;              /* 方向 */
  cmd[4] = acc;              /* 加速度，注意：0是直接启动 */
  cmd[5] = (uint8_t)((speed >> 8) & 0xFF);   /* 速度(RPM)高8位字节 */
  cmd[6] = (uint8_t)((speed >> 0) & 0xFF);   /* 速度(RPM)低8位字节 */
  cmd[7] = (uint8_t)((pulses >> 24) & 0xFF); /* 脉冲数(bit24 - bit31 ) */
  cmd[8] = (uint8_t)((pulses >> 16) & 0xFF); /* 脉冲数(bit16 - bit23) */
  cmd[9] = (uint8_t)((pulses >> 8) & 0xFF);  /* 脉冲数(bit8  - bit15) */
  cmd[10] = (uint8_t)((pulses >> 0) & 0xFF); /* 脉冲数(bit0  - bit7) */
  cmd[11] = smd_checksum(cmd, 11);           /* 校验和 */
  cmd[12] = FRAME_TAIL;                      /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 13);
}

/**
 * @brief    脉冲模式
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pulse_mode(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_PULSES_MODE;      /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    脉宽位置模式
 * @param    addr     :  电机地址
 * @param    topw_max :  高电平脉宽最长长度(0 < topw_max < 50000us)
 * @param    topw_min :  高电平脉宽最小长度(0 < topw_min < 50000us)
 * @param    top_pos :   对应的位置(51200为一圈)
 * @param    down_pos :  脉宽最小长度对应的位置
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pulse_width_pos_mode(uint8_t addr, uint16_t topw_max,
                              uint16_t topw_min, int32_t top_pos,
                              int32_t down_pos) {
  uint8_t cmd[32] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;               /* 帧头 */
  cmd[1] = addr;                     /* 地址 */
  cmd[2] = FCT_PULSE_WIDTH_POS_MODE; /* 功能码 */
  cmd[3] =
      (uint8_t)((topw_max >> 8) & 0xFF); /* 高电平脉宽最长长度高8位字节 us */
  cmd[4] = (uint8_t)((topw_max >> 0) & 0xFF); /* 高电平脉宽最长长度低8位字节 */
  cmd[5] =
      (uint8_t)((topw_min >> 8) & 0xFF); /* 高电平脉宽最小长度高8位字节 us */
  cmd[6] = (uint8_t)((topw_min >> 0) & 0xFF); /* 高电平脉宽最小长度低8位字节 */
  /* 高电平脉宽最大长度对应的位置 */
  cmd[7] = (uint8_t)((top_pos >> 24) & 0xFF); /* (bit24 - bit31 ) */
  cmd[8] = (uint8_t)((top_pos >> 16) & 0xFF); /* (bit16 - bit23) */
  cmd[9] = (uint8_t)((top_pos >> 8) & 0xFF);  /* (bit8  - bit15) */
  cmd[10] = (uint8_t)((top_pos >> 0) & 0xFF); /* (bit0  - bit7) */
  /* 高电平脉宽最小长度对应的位置 */
  cmd[11] = (uint8_t)((down_pos >> 24) & 0xFF); /* (bit24 - bit31 ) */
  cmd[12] = (uint8_t)((down_pos >> 16) & 0xFF); /* (bit16 - bit23) */
  cmd[13] = (uint8_t)((down_pos >> 8) & 0xFF);  /* (bit8  - bit15) */
  cmd[14] = (uint8_t)((down_pos >> 0) & 0xFF);  /* (bit0  - bit7) */
  cmd[15] = smd_checksum(cmd, 15);              /* 校验和 */
  cmd[16] = FRAME_TAIL;                         /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 17);
}

/**
 * @brief    脉宽电流模式
 * @param    addr     :  电机地址
 * @param    topw_max :  高电平脉宽最长长度(0 < topw_max < 50000us)
 * @param    topw_min :  高电平脉宽最小长度(0 < topw_min < 50000us)
 * @param    top_pos :   脉宽最长长度对应的电流（单位mA）
 * @param    down_pos :  脉宽最小长度对应的电流
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pulse_width_ma_mode(uint8_t addr, uint16_t topw_max, uint16_t topw_min,
                             int32_t top_ma, int32_t down_ma) {
  uint8_t cmd[32] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;              /* 帧头 */
  cmd[1] = addr;                    /* 地址 */
  cmd[2] = FCT_PULSE_WIDTH_MA_MODE; /* 功能码 */
  cmd[3] =
      (uint8_t)((topw_max >> 8) & 0xFF); /* 高电平脉宽最长长度高8位字节 us */
  cmd[4] = (uint8_t)((topw_max >> 0) & 0xFF); /* 高电平脉宽最长长度低8位字节 */
  cmd[5] =
      (uint8_t)((topw_min >> 8) & 0xFF); /* 高电平脉宽最小长度高8位字节 us */
  cmd[6] = (uint8_t)((topw_min >> 0) & 0xFF); /* 高电平脉宽最小长度低8位字节 */
  /* 高电平脉宽最大长度对应的电流 */
  cmd[7] = (uint8_t)((top_ma >> 24) & 0xFF); /* (bit24 - bit31 ) */
  cmd[8] = (uint8_t)((top_ma >> 16) & 0xFF); /* (bit16 - bit23) */
  cmd[9] = (uint8_t)((top_ma >> 8) & 0xFF);  /* (bit8  - bit15) */
  cmd[10] = (uint8_t)((top_ma >> 0) & 0xFF); /* (bit0  - bit7) */
  /* 高电平脉宽最小长度对应的电流 */
  cmd[11] = (uint8_t)((down_ma >> 24) & 0xFF); /* (bit24 - bit31 ) */
  cmd[12] = (uint8_t)((down_ma >> 16) & 0xFF); /* (bit16 - bit23) */
  cmd[13] = (uint8_t)((down_ma >> 8) & 0xFF);  /* (bit8  - bit15) */
  cmd[14] = (uint8_t)((down_ma >> 0) & 0xFF);  /* (bit0  - bit7) */
  cmd[15] = smd_checksum(cmd, 15);             /* 校验和 */
  cmd[16] = FRAME_TAIL;                        /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 17);
}

/**
 * @brief    脉宽速度模式
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_pulse_width_speed_mode(uint8_t addr, uint16_t topw_max,
                                uint16_t topw_min, int32_t top_speed,
                                int32_t down_speed) {
  uint8_t cmd[32] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;                 /* 帧头 */
  cmd[1] = addr;                       /* 地址 */
  cmd[2] = FCT_PULSE_WIDTH_SPEED_MODE; /* 功能码 */
  cmd[3] =
      (uint8_t)((topw_max >> 8) & 0xFF); /* 高电平脉宽最长长度高8位字节 us */
  cmd[4] = (uint8_t)((topw_max >> 0) & 0xFF); /* 高电平脉宽最长长度低8位字节 */
  cmd[5] =
      (uint8_t)((topw_min >> 8) & 0xFF); /* 高电平脉宽最小长度高8位字节 us */
  cmd[6] = (uint8_t)((topw_min >> 0) & 0xFF); /* 高电平脉宽最小长度低8位字节 */
  /* 高电平脉宽最大长度对应的速度 */
  cmd[7] = (uint8_t)((top_speed >> 24) & 0xFF); /* (bit24 - bit31 ) */
  cmd[8] = (uint8_t)((top_speed >> 16) & 0xFF); /* (bit16 - bit23) */
  cmd[9] = (uint8_t)((top_speed >> 8) & 0xFF);  /* (bit8  - bit15) */
  cmd[10] = (uint8_t)((top_speed >> 0) & 0xFF); /* (bit0  - bit7) */
  /* 高电平脉宽最小长度对应的速度 */
  cmd[11] = (uint8_t)((down_speed >> 24) & 0xFF); /* (bit24 - bit31 ) */
  cmd[12] = (uint8_t)((down_speed >> 16) & 0xFF); /* (bit16 - bit23) */
  cmd[13] = (uint8_t)((down_speed >> 8) & 0xFF);  /* (bit8  - bit15) */
  cmd[14] = (uint8_t)((down_speed >> 0) & 0xFF);  /* (bit0  - bit7) */
  cmd[15] = smd_checksum(cmd, 15);                /* 校验和 */
  cmd[16] = FRAME_TAIL;                           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 17);
}

/**
 * @brief    开环速度模式
 * @param    addr      :  电机地址
 * @param    dir       :  方向       ，0正转，1反转
 * @param    acc       :  加速度     ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed     :  速度       ，范围0.1 - 3000RPM
 * @retval   从机应答   :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_ol_speed_mode(uint8_t addr, uint8_t dir, uint8_t acc, float speed) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;        /* 帧头 */
  cmd[1] = addr;              /* 地址 */
  cmd[2] = FCT_OL_SPEED_MODE; /* 功能码 */
  cmd[3] = dir;               /* 方向 */
  cmd[4] = acc;               /* 加速度，注意：0是直接启动 */
  data_u.f = speed;           /* 速度(RPM) */
  cmd[5] = data_u.b[3];
  cmd[6] = data_u.b[2];
  cmd[7] = data_u.b[1];
  cmd[8] = data_u.b[0];
  cmd[9] = smd_checksum(cmd, 9); /* 校验和 */
  cmd[10] = FRAME_TAIL;          /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 11);
}

/**
 * @brief    开环绝对位置模式控制
 * @param    addr     :  电机地址
 * @param    dir      :  方向        0为CW(顺时针)，其余为CCW(逆时针)
 * @param    acc      :  加速度      ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed    :  最大速度    ，范围0 - 3000RPM
 * @param    pulses   :  脉冲数      ，范围0- (2^32 - 1)个
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_ol_pos_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed,
                     uint32_t pulses) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;      /* 帧头 */
  cmd[1] = addr;            /* 地址 */
  cmd[2] = FCT_OL_POS_MODE; /* 功能码 */
  cmd[3] = dir;             /* 方向 */
  cmd[4] = acc;             /* 加速度，注意：0是直接启动 */
  cmd[5] = (uint8_t)((speed >> 8) & 0xFF);   /* 速度(RPM)高8位字节 */
  cmd[6] = (uint8_t)((speed >> 0) & 0xFF);   /* 速度(RPM)低8位字节 */
  cmd[7] = (uint8_t)((pulses >> 24) & 0xFF); /* 脉冲数(bit24 - bit31 ) */
  cmd[8] = (uint8_t)((pulses >> 16) & 0xFF); /* 脉冲数(bit16 - bit23) */
  cmd[9] = (uint8_t)((pulses >> 8) & 0xFF);  /* 脉冲数(bit8  - bit15) */
  cmd[10] = (uint8_t)((pulses >> 0) & 0xFF); /* 脉冲数(bit0  - bit7) */
  cmd[12] = smd_checksum(cmd, 12);           /* 校验和 */
  cmd[13] = FRAME_TAIL;                      /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 14);
}

/**
 * @brief    开环相对位置模式控制
 * @param    addr     :  电机地址
 * @param    dir      :  方向        0为CW(顺时针)，其余为CCW(逆时针)
 * @param    acc      :  加速度      ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed    :  最大速度    ，范围0 - 3000RPM
 * @param    pulses   :  脉冲数      ，范围0- (2^32 - 1)个
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_ol_pos_rel_mode(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed,
                         uint32_t pulses) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;          /* 帧头 */
  cmd[1] = addr;                /* 地址 */
  cmd[2] = FCT_OL_POS_REL_MODE; /* 功能码 */
  cmd[3] = dir;                 /* 方向 */
  cmd[4] = acc;                 /* 加速度，注意：0是直接启动 */
  cmd[5] = (uint8_t)((speed >> 8) & 0xFF);   /* 速度(RPM)高8位字节 */
  cmd[6] = (uint8_t)((speed >> 0) & 0xFF);   /* 速度(RPM)低8位字节 */
  cmd[7] = (uint8_t)((pulses >> 24) & 0xFF); /* 脉冲数(bit24 - bit31 ) */
  cmd[8] = (uint8_t)((pulses >> 16) & 0xFF); /* 脉冲数(bit16 - bit23) */
  cmd[9] = (uint8_t)((pulses >> 8) & 0xFF);  /* 脉冲数(bit8  - bit15) */
  cmd[10] = (uint8_t)((pulses >> 0) & 0xFF); /* 脉冲数(bit0  - bit7) */
  cmd[12] = smd_checksum(cmd, 12);           /* 校验和 */
  cmd[13] = FRAME_TAIL;                      /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 14);
}

/**
 * @brief    开环脉冲模式
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_ol_pulse_mode(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_OL_PULSES_MODE;   /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    IO启停模式
 * @param    addr      :  电机地址
 * @param    dir       :  方向       ，0正转，1反转
 * @param    acc       :  加速度     ，范围0 - 200，单位RPM/SS 注意：0直接启动
 * @param    speed     :  速度       ，范围0.1 - 3000RPM
 * @retval   从机应答   :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_io_run_ctrl(uint8_t addr, uint8_t dir, uint8_t acc, float speed) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;      /* 帧头 */
  cmd[1] = addr;            /* 地址 */
  cmd[2] = FCT_IO_RUN_MODE; /* 功能码 */
  cmd[3] = dir;             /* 方向 */
  cmd[4] = acc;             /* 加速度，注意：0是直接启动 */
  data_u.f = speed;         /* 速度(RPM) */
  cmd[5] = data_u.b[3];
  cmd[6] = data_u.b[2];
  cmd[7] = data_u.b[1];
  cmd[8] = data_u.b[0];
  cmd[9] = smd_checksum(cmd, 9); /* 校验和 */
  cmd[10] = FRAME_TAIL;          /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 11);
}

/**
 * @brief    将当前位置角度清零
 * @param    addr    :  电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 命令状态 + 校验字节 + 帧尾
 */
void smd_angle_to_zero(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_ANGLE_ZERO;       /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    解除堵转保护
 * @param    addr     : 电机地址
 * @retval   从机应答  : 帧头 + 地址 + 功能码 + 命令状态 + 校验字节 + 帧尾
 */
void smd_remove_clog_protect(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_CLEAR_CLOG_PRO;   /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    电机使能控制
 * @param    addr     :  电机地址
 * @param    en       :  0 使能电机， 1 失能
 * @retval   从机应答 :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_motor_enable(uint8_t addr, uint8_t en) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_MOTOR_ENABLE;     /* 功能码 */
  cmd[3] = en;                   /* 使能模式 */
  cmd[4] = smd_checksum(cmd, 4); /* 校验和 */
  cmd[5] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 6);
}

/**
 * @brief    清除电机状态（堵转、失能、刹车）
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_clear_sta(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_CLEAR_STATE;      /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}

/**
 * @brief    立即停止
 * @param    addr     :  电机地址
 * @retval   从机应答  :  帧头 + 地址 + 功能码 + 参数列表 + 校验字节 + 帧尾
 */
void smd_stop_now(uint8_t addr) {
  uint8_t cmd[16] = {0};

  /* 装载命令 */
  cmd[0] = FRAME_HEAD;           /* 帧头 */
  cmd[1] = addr;                 /* 地址 */
  cmd[2] = FCT_STOP_NOW;         /* 功能码 */
  cmd[3] = smd_checksum(cmd, 3); /* 校验和 */
  cmd[4] = FRAME_TAIL;           /* 帧尾 */

  /* 发送命令 */
  smd_send_data(cmd, 5);
}
