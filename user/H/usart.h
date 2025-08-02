#ifndef __USART_H
#define __USART_H
#include "public.h"
#define MODBUS_BUFFER_SIZE 64  // 增加到64字节，支持更大的帧
#define MODBUS_SLAVE_ID 66
#define MODBUS_TIMEOUT_MS 5000 // 5秒超时

// 日志相关定义
#define LOG_BUFFER_SIZE 20    // 保存20条日志
#define LOG_DATA_SIZE 32      // 每条日志最大32字节

typedef struct {
    uint32_t timestamp;       // 时间戳
    uint8_t level;           // 日志级别 (0:INFO, 1:WARN, 2:ERROR)
    uint8_t length;          // 日志数据长度
    uint8_t data[LOG_DATA_SIZE]; // 日志数据
} LogEntry_t;

void Log_Write(uint8_t level, const uint8_t *data, uint8_t length);

extern u8 receive_count;
extern u8 receive[MODBUS_BUFFER_SIZE];
extern u8 receive_flag;
extern uint8_t relay1_allow;
extern uint8_t relay2_allow;
void USART1_Configuration(void);
void Receive_Data(void);
void usart1_send_one_byte(uchar data);
void usart1_send_string(const char *buf);
typedef unsigned char bool;
bool Modbus_Check_Timeout(void);
#endif


