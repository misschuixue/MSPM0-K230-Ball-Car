/**
 * @file process_frame.c
 * @brief 底层驱动与业务逻辑模块
 * @details 该文件由 TeamSpark 于 2026 NUEDC 备赛期间重构，符合统一编码规范。
 * @author TeamSpark
 * @date 2026-07-10
 */
#include "process_frame.h"
#include "usart.h"
#include "smd.h"
#include "string.h"

/* 将读取到的数据转成浮点数 */
typedef union
{
    float f;
    uint8_t b[4];
} float_union_t;


/**
 * @brief       将 4 字节的数据转换为 float 浮点数
 * @param       buf     指向包含4字节数据的缓冲区（小端格式：低字节在前）
 * @return      转换后的 float 浮点数
 */
float bytes_to_float(uint8_t *buf)
{
    float_union_t u;

    /* 将字节逐一写入联合体中的字节数组（假设小端顺序） */
    u.b[0] = buf[3];   /* 低字节 */
    u.b[1] = buf[2];
    u.b[2] = buf[1];
    u.b[3] = buf[0];   /* 高字节 */

    /* 返回组合后的浮点数 */
    return u.f;
}

/**
 * @brief   串口数据解析
 * @param   buffer: 输入缓冲区
 * @param   len: 缓冲区长度
 * @param   frame: 输出解析结果
 * @retval  解析是否成功
 */
bool serial_frame_process(uint8_t *buffer, uint8_t len, SERIAL_FRAME *frame)
{
    /* 初始化输出结构 */
    memset(frame, 0, sizeof(SERIAL_FRAME));

    /* 填充解析结果 */
    frame->slave_addr = buffer[1];
    frame->function_code = buffer[2];
    frame->error_code = buffer[3];
    frame->checksum = buffer[len-2];
    
    /* 计算数据部分长度（排除固定部分） */
    frame->data_len = len - 6;  /* 总长 - (固定部分：头(1) + 地址(1) + 功能码(1) + 错误码（1） + 校验和(1) + 尾(1) = 6字节) */

    if (frame->data_len > 0 && frame->data_len <= sizeof(frame->data))
    {
        memcpy(frame->data, &buffer[4], frame->data_len);
    }
    else
    {
        frame->data_len = 0;
    }
    if(frame->error_code != ACK_SUCCEED)
    {
        /* 根据错误码处理 */
        switch(frame->error_code)
        {
            case ACK_FRAME_TOO_SHORT: 
                printf("帧长度不足\r\n");
                break;
            case ACK_INVALID_HEADER: 
                printf("无效帧头 \r\n");
                break;
            case ACK_INVALID_FOOTER: 
                printf("无效帧尾 \r\n");
                break;
            case ACK_CHECKSUM_MISMATCH: 
                printf("校验和错误\r\n");
                break;
            case ACK_UNSUPPORTED_FUNCTION: 
                printf("不支持的功能码\r\n");
                break;
            case ACK_ERR_ILLEGAL_VAL: 
                printf("数据不合法\r\n");
                break;
            default : break;
            
        }
        return false;
    }

    /* 根据功能码处理 */
    switch(frame->function_code)
    {
        case FCT_CAL_ENCODER:
            if(frame->data[0] == 1) printf("校准中 \r\n");
            else if(frame->data[0] == 2) printf("校准失败 \r\n");
            else if(frame->data[0] == 3) printf("校准成功 \r\n");
            break;
        
        case FCT_RESTART:
            printf("复位成功\r\n");
            break;
        
        case FCT_RESET_FACTORY:
            printf("恢复出厂设置成功，请等待重新识别参数\r\n");
            break;
        
        case FCT_PARAM_SAVE:
            printf("参数保存成功\r\n");
            break;
                
        case FCT_READ_SOFT_HARD_VER:
            printf("软件版本:V%d.%d, 硬件版本:V%d.%d\r\n",frame->data[0]/10,frame->data[0]%10,frame->data[1]/10,frame->data[1]%10);
            break;
        
        case FCT_READ_PSI:
        {
            float psi = bytes_to_float((uint8_t *)&frame->data[0]);
            printf("磁链：%.2fmWb\r\n", psi);
            break;
        }
        case FCT_READ_PHASE_RES_IND:
        {
            float rs = bytes_to_float((uint8_t *)&frame->data[0]);
            printf("相电阻：%.2fΩ\r\n", rs);
            float ls = bytes_to_float((uint8_t *)&frame->data[4]);
            printf("相电感：%.2fmH\r\n", ls);
            break;
        }
 
        case FCT_READ_PHASE_MA:
        {
            int16_t iq = (int16_t)((int16_t)frame->data[0]<< 8 | frame->data[1]);
            printf("相电流：%dmA\r\n", iq);
            break;
        }

        case FCT_READ_VOL:
        {
            float power = bytes_to_float((uint8_t *)&frame->data[0]);
            printf("输入电压：%.1fV\r\n", power);
            break;
        }
        
        case FCT_READ_MA_PID:
        {
            float q_kp = bytes_to_float((uint8_t *)&frame->data[0]);
            float q_ki = bytes_to_float((uint8_t *)&frame->data[4]);
            float d_kp = bytes_to_float((uint8_t *)&frame->data[8]);
            float d_ki = bytes_to_float((uint8_t *)&frame->data[12]);
            printf("电流环DQ轴PI参数：q_kp:%.5f, q_ki:%.5f, d_kp:%.5f, d_ki:%.5f\r\n", q_kp, q_ki, d_kp, d_ki);
            break;
        }
        
        case FCT_READ_SPEED_PID:
        {
            uint32_t kp = (uint32_t)((uint32_t)frame->data[0] << 24 | (uint32_t)frame->data[1] << 16 | (uint32_t)frame->data[2] << 8 | (uint32_t)frame->data[3] << 0);
            uint32_t ki = (uint32_t)((uint32_t)frame->data[4] << 24 | (uint32_t)frame->data[5] << 16 | (uint32_t)frame->data[6] << 8 | (uint32_t)frame->data[7] << 0);
            uint32_t kd = (uint32_t)((uint32_t)frame->data[8] << 24 | (uint32_t)frame->data[9] << 16 | (uint32_t)frame->data[10] << 8 | (uint32_t)frame->data[11] << 0);
            printf("速度环PID参数：kp:%d, ki:%d, kd:%d\r\n", kp, ki, kd);
            break;
        }
        
        case FCT_READ_POS_PID:
        {
            uint32_t kp = (uint32_t)((uint32_t)frame->data[0] << 24 | (uint32_t)frame->data[1] << 16 | (uint32_t)frame->data[2] << 8 | (uint32_t)frame->data[3] << 0);
            uint32_t ki = (uint32_t)((uint32_t)frame->data[4] << 24 | (uint32_t)frame->data[5] << 16 | (uint32_t)frame->data[6] << 8 | (uint32_t)frame->data[7] << 0);
            uint32_t kd = (uint32_t)((uint32_t)frame->data[8] << 24 | (uint32_t)frame->data[9] << 16 | (uint32_t)frame->data[10] << 8 | (uint32_t)frame->data[11] << 0);
            printf("位置环PID参数：kp:%d, ki:%d, kd:%d\r\n", kp, ki, kd);
            break;
        }
        case FCT_READ_TOTAL_PULSE:
        {
            int32_t pulse_cnt = (int32_t)((int32_t)frame->data[0] << 24 | (int32_t)frame->data[1] << 16 | (int32_t)frame->data[2] << 8 | (int32_t)frame->data[3] << 0);
            printf("总脉冲数：%d\r\n", pulse_cnt);
            break;
        }
        
        case FCT_READ_ROTATE_SPEED:
        {
            int16_t rpm = (int16_t)((int16_t)frame->data[0] << 8 | (int16_t)frame->data[1] << 0);
            printf("实时转速：%dRPM\r\n", rpm);
            break;
        }
        
        case FCT_READ_POS:
        {
            int32_t pos = (int32_t)((int32_t)frame->data[0] << 24 | (int32_t)frame->data[1] << 16 | (int32_t)frame->data[2] << 8 | (int32_t)frame->data[3] << 0);
            extern uint8_t g_smd_target_uart;
            printf("步进电机%d位置：%d\r\n", g_smd_target_uart, pos);
            break;
        }
        
        case FCT_READ_POS_ERROR:
        {
            int32_t pos_err = (int32_t)((int32_t)frame->data[0] << 24 | (int32_t)frame->data[1] << 16 | (int32_t)frame->data[2] << 8 | (int32_t)frame->data[3] << 0);
            printf("位置误差：%d\r\n", pos_err);
            break;
        }
        
        case FCT_READ_MOTOR_STA:
        {
            if(frame->data[0] == 0) printf("电机状态：停止\r\n");
            else if(frame->data[0] == 1) printf("电机状态：闭环完成\r\n");
            else if(frame->data[0] == 2) printf("电机状态：闭环运行中\r\n");
            else if(frame->data[0] == 3) printf("电机状态：故障\r\n");
            else if(frame->data[0] == 4) printf("电机状态：堵转\r\n");
            else if(frame->data[0] == 5) printf("电机状态：欠压\r\n");            
            break;
        }
        
        case FCT_READ_CLOG_FLAG:
        {
            if(frame->data[0] == 0) printf("未堵转\r\n");
            else if(frame->data[0] == 1) printf("堵转\r\n");           
            break;
        }
        
        case FCT_READ_CLOG_CUR:
        {
            int16_t stall_ma = (int16_t)((int16_t)frame->data[0]<< 8 | (int16_t)frame->data[1] << 0);
            printf("堵转电流：%dmA\r\n", stall_ma);
            break;
        }
        
        case FCT_READ_ENABLE_STA:
        {
            if(frame->data[0] == 0) printf("使能\r\n");
            else if(frame->data[0] == 1) printf("失能\r\n");
            break;
        }
        
        case FCT_READ_ARRIVED_STA:
        {
            if(frame->data[0] == 0) printf("未到位\r\n");
            else if(frame->data[0] == 1) printf("到位\r\n");
            break;
        }
        
        case FCT_READ_SYS_PARAM:
        {
            float power = bytes_to_float((uint8_t *)&frame->data[0]);
            printf("输入电压：%.1fV\r\n", power);
            
            int16_t iq = (int16_t)((int16_t)frame->data[4] << 8  | (int16_t)frame->data[5] << 0);
            printf("相电流：%dmA\r\n", iq);
            
            float psi = bytes_to_float((uint8_t *)&frame->data[6]);
            printf("磁链：%.2fmWb\r\n", psi);
            
            float rs = bytes_to_float((uint8_t *)&frame->data[10]);
            printf("相电阻：%.2fΩ\r\n", rs);
            
            float ls = bytes_to_float((uint8_t *)&frame->data[14]);
            printf("相电感：%.2fmH\r\n", ls);
            
            int16_t rpm = (int16_t)((int16_t)frame->data[18] << 8 | (int16_t)frame->data[19] << 0);
            printf("实时转速：%dRPM\r\n", rpm);
            
            int32_t pos_tar = (int32_t)((int32_t)frame->data[20] << 24 | (int32_t)frame->data[21] << 16 | (int32_t)frame->data[22] << 8 | (int32_t)frame->data[23] << 0);
            printf("目标位置：%d\r\n", pos_tar);
            
            int32_t pos = (int32_t)((int32_t)frame->data[24] << 24 | (int32_t)frame->data[25] << 16 | (int32_t)frame->data[26] << 8 | (int32_t)frame->data[27] << 0);
            printf("实时位置：%d\r\n", pos);
            
            int32_t pos_err = (int32_t)((int32_t)frame->data[28] << 24 | (int32_t)frame->data[29] << 16 | (int32_t)frame->data[30] << 8 | (int32_t)frame->data[31] << 0);
            printf("位置误差：%d\r\n", pos_err);
            
            int32_t pulse_cnt = (int32_t)((int32_t)frame->data[32] << 24 | (int32_t)frame->data[33] << 16 | (int32_t)frame->data[34] << 8 | (int32_t)frame->data[35] << 0);
            printf("总脉冲数：%d\r\n", pulse_cnt);
            if(frame->data[36]) printf("电机失能\r\n");
            else printf("电机使能\r\n");
            if(frame->data[37]) printf("电机到位\r\n");
            else printf("电机未到位\r\n");
            if(frame->data[38]) printf("电机堵转\r\n");
            else printf("电机未堵转\r\n");
            
            if(frame->data[39]) printf("分组模式（控制多机）\r\n");
            else printf("独立模式（控制单机）\r\n");
            break;
        }
        case FCT_READ_DRIVE_PARAMS:
        {       
            if(frame->data[0] == 0) printf("开环位置模式\r\n");
            else if(frame->data[0] == 1) printf("开环速度模式\r\n");
            else if(frame->data[0] == 2) printf("开环力矩模式\r\n");
            else if(frame->data[0] == 3) printf("脉冲模式\r\n");    
            else if(frame->data[0] == 4) printf("闭环位置模式\r\n");
            else if(frame->data[0] == 5) printf("闭环速度模式\r\n"); 
            else if(frame->data[0] == 6) printf("闭环力矩模式\r\n");
            else if(frame->data[0] == 7) printf("回零模式\r\n");   
            else if(frame->data[0] == 8) printf("开环速度模式\r\n"); 
            else if(frame->data[0] == 9) printf("开环位置模式\r\n"); 
            else if(frame->data[0] == 10) printf("开环脉冲模式\r\n"); 
            
            if(frame->data[1]) printf("指令不回响\r\n");
            else printf("指令正常回响\r\n");
            
            uint32_t uart_baud = (uint32_t)((uint32_t)frame->data[2] << 24 | (uint32_t)frame->data[3] << 16 | (uint32_t)frame->data[4] << 8 | (uint32_t)frame->data[5] << 0);
            printf("串口波特率为：%d\r\n", uart_baud);
            
            uint16_t can_baud = (uint16_t)((uint16_t)frame->data[6] << 8 | (uint16_t)frame->data[7] << 0);
            printf("CAN速率为：%dK\r\n", can_baud);
            
            if(frame->data[8]) printf("DIR低电平正转\r\n");
            else printf("DIR高电平正转\r\n");
            
            if(frame->data[9] == 0) printf("EN脚低电平有效\r\n");
            else if(frame->data[9] == 1) printf("EN脚高电平有效\r\n");
            else if(frame->data[9] == 2) printf("EN保持有效\r\n");
            
            uint16_t step = (uint16_t)((uint16_t)frame->data[10] << 8 | (uint16_t)frame->data[11] << 0);
            printf("细分为：%d\r\n", step);
            
            int16_t pos_ma = (int16_t)((int32_t)frame->data[12] << 8 | (int32_t)frame->data[13] << 0);
            printf("最大位置：%d\r\n", pos_ma);
            
            uint32_t l_kp = (uint32_t)((uint32_t)frame->data[14] << 24 | (uint32_t)frame->data[15] << 16 | (uint32_t)frame->data[16] << 8 | (uint32_t)frame->data[17] << 0);
            uint32_t l_ki = (uint32_t)((uint32_t)frame->data[18] << 24 | (uint32_t)frame->data[19] << 16 | (uint32_t)frame->data[20] << 8 | (uint32_t)frame->data[21] << 0);
            uint32_t l_kd = (uint32_t)((uint32_t)frame->data[22] << 24 | (uint32_t)frame->data[23] << 16 | (uint32_t)frame->data[24] << 8 | (uint32_t)frame->data[25] << 0);
            printf("位置环PID参数：kp:%d, ki:%d, kd:%d\r\n", l_kp, l_ki, l_kd);
                       
            uint16_t stall_ma = (int16_t)((int16_t)frame->data[26] << 8 | (int16_t)frame->data[27] << 0);
            printf("堵转保护电流为：%dmA\r\n", stall_ma);
            
            if(frame->data[28]) printf("开启堵转保护功能\r\n");
            else printf("关闭堵转保护功能\r\n");
            
            if(frame->data[29]) printf("按键上锁\r\n");
            else printf("按键解锁\r\n");
            
            uint32_t s_kp = (uint32_t)((uint32_t)frame->data[30] << 24 | (uint32_t)frame->data[31] << 16 | (uint32_t)frame->data[32] << 8 | (uint32_t)frame->data[33] << 0);
            uint32_t s_ki = (uint32_t)((uint32_t)frame->data[34] << 24 | (uint32_t)frame->data[35] << 16 | (uint32_t)frame->data[36] << 8 | (uint32_t)frame->data[37] << 0);
            uint32_t s_kd = (uint32_t)((uint32_t)frame->data[38] << 24 | (uint32_t)frame->data[39] << 16 | (uint32_t)frame->data[40] << 8 | (uint32_t)frame->data[41] << 0);
            printf("速度环PID参数：kp:%d, ki:%d, kd:%d\r\n", s_kp, s_ki, s_kd);
            
            if(frame->data[42]) printf("开启自动熄屏\r\n");
            else printf("关闭自动熄屏\r\n");
            
            if(frame->data[43]) printf("IO启停高电平启动\r\n");
            else printf("IO启停低电平启动\r\n");
            break;
        }

        case FCT_SET_SLAVE_ADD:
            printf("成功设置从机地址为：0x%x\r\n",frame->data[0]);
            break;
        
        case FCT_SET_GROUP_ADD:
            printf("成功设置分组地址为：0x%x\r\n",frame->data[0]);
            break;
        
        case FCT_SET_MODE:
            if(frame->data[0] == 0) printf("成功设置电机工作模式为：开环位置模式\r\n");
            else if(frame->data[0] == 1) printf("成功设置电机工作模式为：开环速度模式\r\n");
            else if(frame->data[0] == 2) printf("成功设置电机工作模式为：开环力矩模式\r\n");
            else if(frame->data[0] == 3) printf("成功设置电机工作模式为：脉冲模式\r\n");    
            else if(frame->data[0] == 4) printf("成功设置电机工作模式为：闭环位置模式\r\n");
            else if(frame->data[0] == 5) printf("成功设置电机工作模式为：闭环速度模式\r\n"); 
            else if(frame->data[0] == 6) printf("成功设置电机工作模式为：闭环力矩模式\r\n");
            else if(frame->data[0] == 7) printf("成功设置电机工作模式为：回零模式\r\n");
            else if(frame->data[0] == 8) printf("成功设置电机工作模式为：开环速度模式\r\n"); 
            else if(frame->data[0] == 9) printf("成功设置电机工作模式为：开环位置模式\r\n"); 
            else if(frame->data[0] == 10) printf("成功设置电机工作模式为：开环脉冲模式\r\n"); 
            break;
        
        case FCT_SET_POS_PID:
            printf("成功设置位置环PID参数 P:%d, I:%d, D:%d\r\n",  (uint32_t)(frame->data[0] << 24 | frame->data[1] << 16 | frame->data[2] << 8 | frame->data[3] << 0),\
                                                                (uint32_t)(frame->data[4] << 24 | frame->data[5] << 16 | frame->data[6] << 8 | frame->data[7] << 0),\
                                                                (uint32_t)(frame->data[8] << 24 | frame->data[9] << 16 | frame->data[10] << 8 | frame->data[11] << 0));
            break;
                
        case FCT_SET_POS_TORQUE:
            printf("成功设置位置环输出力矩为：%dmA\r\n",(int16_t)(frame->data[0] << 8 | frame->data[1] << 0));
            break;

        case FCT_SET_STEP:
            printf("成功设置细分为：%d\r\n",(uint16_t)(frame->data[0] << 8 | frame->data[1] << 0));
            break;
        
        case FCT_SET_MA:
            printf("成功设置最大电流限制为：%dmA\r\n",(int16_t)(frame->data[0] << 8 | frame->data[1] << 0));
            break;
        
        case FCT_SET_UART_BAUD:
            printf("成功设置串口波特率为：%d\r\n",(uint32_t)(frame->data[0] << 24 | frame->data[1] << 16 | frame->data[2] << 8 | frame->data[3] << 0));
            break;
        
        case FCT_SET_CAN_BAUD:
            printf("成功设置CAN波特率为：%dK\r\n",(uint16_t)(frame->data[0] << 8 | frame->data[1] << 0));
            break;
                
        case FCT_SET_MODBUS:
            if(frame->data[0] == 0) printf("使用自定义协议\r\n");
            else if(frame->data[0] == 1) printf("使用modbus协议\r\n");
            break;
        
        case FCT_SET_CLOG_PRO:
            if(frame->data[0] == 0) printf("成功关闭堵转保护\r\n");
            else if(frame->data[0] == 1) printf("成功开启堵转保护\r\n");
            break;
        
        case FCT_SET_CLOG_CUR:
            printf("成功设置堵转保护电流为：%dmA\r\n",(int16_t)(frame->data[0] << 8 | frame->data[1] << 0));
            break;

        case FCT_SET_CAN_ID:
            printf("设置CAN发送ID为：0x%x\r\n",(uint32_t)(frame->data[0] << 24 | frame->data[1] << 16 | frame->data[2] << 8 | frame->data[3] << 0));
            break;

        case FCT_SET_DIR_LEVEL:
            if(frame->data[0] == 0) printf("成功设置旋转方向为：高电平正转\r\n");
            else if(frame->data[0] == 1) printf("成功设置旋转方向为：高电平反转\r\n");
            break;
        
        case FCT_SET_EN_LEVEL:
            if(frame->data[0] == 0) printf("成功设置EN脚低电平有效\r\n");
            else if(frame->data[0] == 1) printf("成功设置EN脚高电平有效\r\n");
            else if(frame->data[0] == 2) printf("成功设置EN脚保持有效\r\n");
            break;
        
        case FCT_SET_KEY_LOCK:
            if(frame->data[0] == 0) printf("成功解锁按键\r\n");
            else if(frame->data[0] == 1) printf("成功上锁按键\r\n");
            break;
        
        case FCT_SET_AUTO_NOT_DISPLAY:
            if(frame->data[0] == 0) printf("成功关闭自动熄屏\r\n");
            else if(frame->data[0] == 1) printf("成功开启自动熄屏\r\n");
            break;
        
        case FCT_SET_IO_START_LEVEL:
            if(frame->data[0] == 0) printf("设置IO低电平启动\r\n");
            else if(frame->data[0] == 1) printf("设置IO高电平启动\r\n");
            break;
        
        case FCT_SET_SPEED_PID:
            printf("成功设置速度环PID参数 P:%d, I:%d, D:%d\r\n",  (uint32_t)(frame->data[0] << 24 | frame->data[1] << 16 | frame->data[2] << 8 | frame->data[3] << 0),\
                                                                (uint32_t)(frame->data[4] << 24 | frame->data[5] << 16 | frame->data[6] << 8 | frame->data[7] << 0),\
                                                                (uint32_t)(frame->data[8] << 24 | frame->data[9] << 16 | frame->data[10] << 8 | frame->data[11] << 0));
            break;
        
        case FCT_ORIGIN_SET_LEFT_POS:
            printf("成功设置左限位原点：%d\r\n",(int32_t)(frame->data[0] << 24 | frame->data[1] << 16 | frame->data[2] << 8 | frame->data[3] << 0));
            break;
        
        case FCT_ORIGIN_LIMIT_HOME:
            if(frame->data[0] == 0) printf("无限位找零\r\n");
            else if(frame->data[0] == 1) printf("有限位找零\r\n");
            break;
        
        case FCT_ORIGIN_TRIG:
            if(frame->data[0] == 0) printf("单次回零\r\n");
            else if(frame->data[0] == 1) printf("就近回零\r\n");
            else if(frame->data[0] == 2) printf("多次回零\r\n");
            break;
         
        case FCT_ORIGIN_BREAK:
            printf("强制退出回零操作成功\r\n");
            break;
        
        case FCT_ORIGIN_READ_PARAMS:
            if(frame->data[0] == 0) printf("上电不自动回零\r\n");
            else if(frame->data[0] == 1) printf("上电自动回零\r\n");
            
            if(frame->data[1] == 0) printf("空闲态\r\n");
            else if(frame->data[1] == 1) printf("找零点中\r\n");
            else if(frame->data[1] == 2) printf("成功找到零点\r\n");
            else if(frame->data[1] == 3) printf("错误没到零点\r\n");
            printf("无限位碰撞到位电流：%d mA\r\n",(int16_t)((frame->data[2] << 8) |frame->data[3]));
            printf("左限位原点：%d\r\n",(int32_t)(frame->data[4] << 24 | frame->data[5] << 16 | frame->data[6] << 8 | frame->data[7] << 0));
            printf("回零超时时间为：%d ms\r\n",(uint32_t)(frame->data[8] << 24 | frame->data[9] << 16 | frame->data[10] << 8 | frame->data[11] << 0));
            printf("右限位原点：%d\r\n",(int32_t)(frame->data[12] << 24 | frame->data[13] << 16 | frame->data[14] << 8 | frame->data[15] << 0));
            if(frame->data[16] == 0) printf("关闭限位保护\r\n");
            else if(frame->data[16] == 1) printf("开启左右限位\r\n");
            break;
        
        case FCT_ORIGIN_SET_PARAMS:
            printf("成功设置找零点超时时间为：%d\r\n",(uint32_t)(frame->data[0] << 24 | frame->data[1] << 16 | frame->data[2] << 8 | frame->data[3] << 0));
            break;
        
        case FCT_ORIGIN_READ_STA:
            if(frame->data[0] == 0) printf("空闲态\r\n");
            else if(frame->data[0] == 1) printf("找零点中\r\n");
            else if(frame->data[0] == 2) printf("成功找到零点\r\n");
            else if(frame->data[0] == 3) printf("错误没到零点\r\n");
            break;
        
        case FCT_ORIGIN_AOTO_ZERO:
            if(frame->data[0] == 0) printf("上电不自动回零\r\n");
            else if(frame->data[0] == 1) printf("上电自动回零\r\n");
            break;
        
        case FCT_ORIGIN_SET_RIGHT_POS:
            printf("成功设置右限位原点：%d\r\n",(int32_t)(frame->data[0] << 24 | frame->data[1] << 16 | frame->data[2] << 8 | frame->data[3] << 0));
            break;
        
        case FCT_ORIGIN_SWITCH:
            if(frame->data[0] == 0) printf("关闭限位保护\r\n");
            else if(frame->data[0] == 1) printf("开启左右限位\r\n");
            break;
        
        case FCT_TORQUE_MODE:
            printf("设置力矩模式成功\r\n");
            break;
        
        case FCT_SPEED_MODE:
            printf("设置速度模式成功\r\n");
            break;
        
        case FCT_POS_MODE:
            printf("设置绝对位置模式成功\r\n");
            break;
        
        case FCT_POS_REL_MODE:
            printf("设置相对位置模式成功\r\n");
            break;
                
        case FCT_PULSES_MODE:
            printf("设置脉冲模式成功\r\n");
            break;
        
        case FCT_PULSE_WIDTH_POS_MODE:
            printf("设置脉宽位置模式成功\r\n");
            break;
        
        case FCT_PULSE_WIDTH_MA_MODE:
             printf("设置脉宽力矩模式成功\r\n");
            break;
        
        case FCT_PULSE_WIDTH_SPEED_MODE:
             printf("设置脉宽速度模式成功\r\n");
            break;
        
        case FCT_OL_SPEED_MODE:
            printf("设置开环速度模式成功\r\n");
            break;
        
        case FCT_OL_POS_MODE:
            printf("设置开环位置模式成功\r\n");
            break;
        
        case FCT_OL_POS_REL_MODE:
            printf("设置开环相对位置模式成功\r\n");
            break;
                
        case FCT_OL_PULSES_MODE:
            printf("设置开环脉冲模式成功\r\n");
            break;
        
        case FCT_IO_RUN_MODE:
            printf("设置IO启停模式成功\r\n");
            break;
        
        case FCT_ANGLE_ZERO:
            printf("清除当前位置成功\r\n");
            break;
        
        case FCT_CLEAR_CLOG_PRO:
            printf("成功清除堵转状态\r\n");
            break;
        
        case FCT_MOTOR_ENABLE:
            if(frame->data[0] == 0) printf("使能电机\r\n");
            else if(frame->data[0] == 1) printf("失能电机\r\n");
            break;
        
        case FCT_CLEAR_STATE:
            printf("成功清除电机状态（如刹车、堵转、失能）\r\n");
            break;
        
        case FCT_STOP_NOW:
            printf("成功刹停\r\n");
            break;

        default: 
            
            return false;
    }
    return true;
}
