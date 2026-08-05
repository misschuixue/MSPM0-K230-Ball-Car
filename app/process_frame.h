/**
 * @file process_frame.h
 * @brief 底层驱动与业务逻辑模块
 * @details ���ļ��� TeamSpark ??2026 NUEDC �����ڼ��ع�������ͳһ����淶??
 * @author TeamSpark
 * @date 2026-07-10
 */
#ifndef __PROCESS_FRAME_H
#define __PROCESS_FRAME_H

#include "stdbool.h"
#include "main.h"

/* 解析帧结构 */
typedef struct
{
    uint8_t slave_addr;      /* 从机地址 */
    uint8_t function_code;   /* 功能码 */
    uint8_t error_code;      /* 错误码 */
    uint8_t data[128];       /* 指令数据缓冲区 */
    uint8_t data_len;        /* 指令数据长度 */
    uint16_t checksum;       /* 校验和 */
} SERIAL_FRAME;

typedef enum
{
    ACK_SUCCEED                 = 0x01,     /* 应答成功 */
    ACK_FRAME_TOO_SHORT         = 0xE1,     /* 帧长度不足（小于最小帧长度） */
    ACK_INVALID_HEADER          = 0xE2,     /* 帧头错误（非0xC5） */
    ACK_INVALID_FOOTER          = 0xE3,     /* 帧尾错误（非0x5C） */
    ACK_CHECKSUM_MISMATCH       = 0xE4,     /* 校验和错误 */
    ACK_UNSUPPORTED_FUNCTION    = 0xE5,     /* 不支持的功能码 */
    ACK_ERR_ILLEGAL_VAL         = 0xE6      /* 数据不合法 */
} ACK_STA;


bool serial_frame_process(uint8_t *buffer, uint8_t len, SERIAL_FRAME *frame);

#endif
