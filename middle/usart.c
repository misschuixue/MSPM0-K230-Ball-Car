/**
 * @file usart.c
 * @brief 串口通信底层驱动与中断服务模块
 * @details 负责 UART0 (K230/日志), UART1 (蓝牙/遥测), UART_MOTOR1/2 (底盘双驱)
 * 的收发及中断解析
 * @author WILLiam
 * @date 2026-06-12
 */

#include "usart.h"
#include "key.h"
#include "protocol.h"
#include "task.h"
#include "timer.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern void K230_Parse_Data(uint8_t byte);

/* ==================== 全局接收缓存与状态 ==================== */

// --- UART_0 (K230 & Debug) ---
uint8_t uart_data = 0;
volatile uint8_t rx_order[] = {
    [0] = 0x00, [1] = 0x00, [2] = 0x00, [3] = 0x00, [4] = 0x00,
    [5] = 0x00, [6] = 0x02, [7] = 0xFF, [8] = 0xFA};
volatile uint8_t last_data = 0x00;
volatile uint8_t pt = 0x00;

// --- UART_1 (Bluetooth / Telemetry) ---
uint8_t uart1_data = 0;
volatile bool g_rx_uart1_flag = false;

// --- UART_MOTOR_1 (左轮电机) ---
volatile bool g_rx_frame_flag = false;
volatile uint8_t g_rx_len = 0;
uint8_t g_rx_cmd[128];

// --- UART_MOTOR_2 (右轮电机) ---
volatile bool g_rx2_frame_flag = false;
volatile uint8_t g_rx2_len = 0;
uint8_t g_rx2_cmd[128];

// --- 其他全局缓存 ---
#define BUFFER_SIZE 32
char serial_buffer[BUFFER_SIZE];
uint8_t buffer_index = 0;
bool command_received = false;

/* ==================== UART 基础发送接口 ==================== */

/**
 * @brief UART_0 阻塞发送单个字符
 */
void uart0_send_char(char ch) {
  uint32_t timeout = 500000;
  // 等待发送 FIFO 有空闲空间，加入超时防止死机
  while ((!DL_UART_isTXFIFOEmpty(UART_0_INST)) && (timeout > 0)) {
    timeout--;
  }
  DL_UART_Main_transmitData(UART_0_INST, ch);
}

/**
 * @brief UART_0 发送多字节数组
 */
void uart0_send_array(uint8_t *buf, int len) {
  for (int i = 0; i < len; i++) {
    uart0_send_char((char)buf[i]);
  }
}

/**
 * @brief UART_0 发送字符串
 */
void uart0_send_string(char *str) {
  while (*str != 0 && str != 0) {
    uart0_send_char(*str++);
  }
}

/**
 * @brief UART_0 发送单字节 (API别名)
 */
void usart0_send_byte(unsigned char byte) {
  uart0_send_char(byte);
}

/**
 * @brief UART_0 发送多字节数组
 */
void usart0_send_bytes(unsigned char *buf, int len) {
  while (len--) {
    uart0_send_char(*buf);
    buf++;
  }
}

/**
 * @brief 移植printf底层依赖
 */
int fputc(int ch, FILE *f) {
  uart0_send_char(ch);
  return ch;
}

/**
 * @brief UART_1 阻塞发送单个字符
 */
void uart1_send_char(char ch) {
  uint32_t timeout = 500000;
  while ((!DL_UART_isTXFIFOEmpty(UART_1_INST)) && (timeout > 0)) {
    timeout--;
  }
  DL_UART_Main_transmitData(UART_1_INST, ch);
}

/**
 * @brief UART_1 发送字符串
 */
void uart1_send_string(char *str) {
  while (*str != 0 && str != 0) {
    uart1_send_char(*str++);
  }
}

/**
 * @brief UART_1 发送单字节
 */
void usart1_send_byte(unsigned char byte) {
  DL_UART_Main_transmitData(UART_1_INST, byte);
}

/**
 * @brief HAL_UART_Transmit兼容接口 (映射至 UART_1)
 */
uint8_t HAL_UART_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout) {
  uint8_t *pdata8bits = pData;
  uint16_t TxXferCount = Size;
  while (TxXferCount > 0U) {
    if (pdata8bits != NULL) {
      DL_UART_Main_transmitData(UART_1_INST, *pdata8bits);
      pdata8bits++;
    }
    TxXferCount--;
    // 等待传输寄存器为空
    uint32_t wait_timeout = 50000;
    while (((UART_STAT_TXFE_MASK & (1 << UART_STAT_TXFF_OFS)) != 0) && (wait_timeout > 0)) {
      wait_timeout--;
    }
  }
  return 1;
}

/* ==================== 格式化打印与遥测接口 ==================== */

/**
 * @brief 带有文件、函数、行号追踪的 Debug 打印函数
 */
int LOG_Debug_Out(const char *__file, const char *__func, int __line,
                  const char *format, ...) {
  va_list args;
  va_start(args, format);

  char log_buff[64] = {0};
  sprintf(log_buff, "[%s Func:%s Line:%d] ", __file, __func, __line);

  char buffer[512] = {0};
  strcpy(buffer, log_buff);
  int len = vsnprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer),
                      format, args);

  va_end(args);

  strcat(buffer, "\r\n");
  uart0_send_string(buffer);

  return len;
}

/**
 * @brief 轻量级串口 0 格式化输出
 */
int lc_printf(char *format, ...) {
  va_list args;
  va_start(args, format);

  char buffer[512] = {0};
  int len = vsnprintf(buffer, sizeof(buffer), format, args);

  va_end(args);

  uart0_send_string(buffer);
  return len;
}

/**
 * @brief 移动端 App 遥测数据发送 (CSV绘图格式)
 */
void Mobile_sendData_UART1(float a, float b, float c) {
  char send_buf[64];

  const char *sign_a = (a < 0) ? "-" : "";
  if (a < 0)
    a = -a;
  int int_a = (int)a;
  int frac_a = (int)((a - int_a) * 100);

  const char *sign_b = (b < 0) ? "-" : "";
  if (b < 0)
    b = -b;
  int int_b = (int)b;
  int frac_b = (int)((b - int_b) * 100);

  const char *sign_c = (c < 0) ? "-" : "";
  if (c < 0)
    c = -c;
  int int_c = (int)c;
  int frac_c = (int)((c - int_c) * 100);

  sprintf(send_buf, "[plot,%s%d.%02d,%s%d.%02d,%s%d.%02d]\r\n", sign_a, int_a,
          frac_a, sign_b, int_b, frac_b, sign_c, int_c, frac_c);

  char *p = send_buf;
  while (*p != '\0') {
    uart1_send_char(*p++);
  }
}

/* ==================== VOFA+ 协议支持接口 ==================== */

typedef union {
  float fdata;
  unsigned long ldata;
} FloatLongType;

void JustFloat_SendArray(uint8_t *string, uint8_t length) {
  while (length--) {
    DL_UART_Main_transmitData(UART_0_INST, *string++);
  }
}

void JustFloat_SendArray_UART1(uint8_t *string, uint8_t length) {
  while (length--) {
    DL_UART_Main_transmitData(UART_1_INST, *string++);
  }
}

void Float_to_Byte(float f, unsigned char byte[]) {
  FloatLongType fl;
  fl.fdata = f;
  byte[0] = (unsigned char)fl.ldata;
  byte[1] = (unsigned char)(fl.ldata >> 8);
  byte[2] = (unsigned char)(fl.ldata >> 16);
  byte[3] = (unsigned char)(fl.ldata >> 24);
}

/* ==================== UART 中断服务函数 ==================== */

/**
 * @brief UART_0 中断服务函数 (K230 数据解析)
 */
void UART_0_INST_IRQHandler(void) {
  switch (DL_UART_getPendingInterrupt(UART_0_INST)) {
  case DL_UART_IIDX_RX:
    uart_data = DL_UART_Main_receiveData(UART_0_INST);
    K230_Parse_Data(uart_data);
    break;
  default:
    break;
  }
}

/**
 * @brief UART_MOTOR_1 中断服务函数 (左轮驱动器通信)
 */
void UART_MOTOR_INST_IRQHandler(void) {
  switch (DL_UART_Main_getPendingInterrupt(UART_MOTOR_INST)) {
  case DL_UART_MAIN_IIDX_RX: {
    uint8_t rx_data = DL_UART_Main_receiveData(UART_MOTOR_INST);

    if (g_rx_frame_flag)
      return;
    if (g_rx_len == 0 && rx_data != 0xC5)
      return; // 帧头过滤

    if (g_rx_len < 128) {
      g_rx_cmd[g_rx_len++] = rx_data;
    } else {
      g_rx_len = 0; // 溢出强制清零
    }

    if (g_rx_len >= 6 && rx_data == 0x5C) {
      g_rx_frame_flag = true;
    }
    break;
  }
  default:
    break;
  }
}

/**
 * @brief UART_MOTOR_2 中断服务函数 (右轮驱动器通信)
 */
void UART_MOTOR_2_INST_IRQHandler(void) {
  switch (DL_UART_Main_getPendingInterrupt(UART_MOTOR_2_INST)) {
  case DL_UART_MAIN_IIDX_RX: {
    uint8_t rx_data = DL_UART_Main_receiveData(UART_MOTOR_2_INST);

    if (g_rx2_frame_flag)
      return;
    if (g_rx2_len == 0 && rx_data != 0xC5)
      return; // 帧头过滤

    if (g_rx2_len < 128) {
      g_rx2_cmd[g_rx2_len++] = rx_data;
    } else {
      g_rx2_len = 0; // 溢出清零
    }

    if (g_rx2_len >= 6 && rx_data == 0x5C) {
      g_rx2_frame_flag = true;
    }
    break;
  }
  default:
    break;
  }
}
