/**
 * @file usart.h
 * @brief 串口通信底层驱动对外接口
 * @author WILLiam
 * @date 2026-06-12
 */

#ifndef __USART_H__
#define __USART_H__

#include "main.h"

/* ==================== 宏定义 ==================== */
#define RX_BUFFER_SIZE 128

/* ==================== 全局变量声明 ==================== */
extern uint8_t uart_data;
extern uint8_t uart1_data;
extern volatile bool g_rx_uart1_flag;

/* ==================== 中断服务函数声明 ==================== */
void UART_MOTOR_INST_IRQHandler(void);

/* ==================== UART 基础发送接口 ==================== */
void uart0_send_char(char ch);
void uart1_send_char(char ch);

void uart0_send_string(char *str);
void uart1_send_string(char *str);

void usart0_send_byte(unsigned char byte);
void usart1_send_byte(unsigned char byte);
void usart0_send_bytes(unsigned char *buf, int len);

uint8_t HAL_UART_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* ==================== VOFA+ 协议支持接口 ==================== */
void JustFloat_SendArray(uint8_t *string, uint8_t length);
void Float_to_Byte(float f, unsigned char byte[]);

/* ==================== 格式化打印与遥测接口 ==================== */
void Mobile_sendData_UART1(float a, float b, float c);

/* 使用可变参数实现的类printf函数 */
int LOG_Debug_Out(const char *__file, const char *__func, int __line,
                  const char *format, ...);

#define LOG_D(fmt, ...)                                                        \
  do {                                                                         \
    LOG_Debug_Out(__FILE__, (const char *)__func__, __LINE__, fmt,             \
                  ##__VA_ARGS__);                                              \
  } while (0)

#endif // __USART_H__
