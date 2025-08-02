#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"
// 外部变量声明
extern uint8_t receive_count;
extern uint8_t modbus_frame_complete;
// 函数声明
void TIM2_Configuration(void);

#endif /* __TIMER_H */
