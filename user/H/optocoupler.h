/**********************************************************************************************************
*@file    optocoupler.h
*@date	  2025-06-21
*@brief	  光耦输入滤波模块
**********************************************************************************************************/
#ifndef __OPTOCOUPLER_H
#define __OPTOCOUPLER_H

#include "stm32f10x.h"

// 添加NULL定义（如果未定义）
#ifndef NULL
#define NULL ((void*)0)
#endif

// 滤波时间定义
#define FILTER_TIME_MS 50  // 滤波时间50毫秒

// 光耦状态变化回调函数类型定义
typedef void (*OPT_StateChangeCallback)(uint8_t new_state);

// 函数声明
void OPT_Init(OPT_StateChangeCallback opt1_callback, OPT_StateChangeCallback opt2_callback);
void OPT_Filter_Update(void);
uint8_t Get_OPT1_Filtered(void);
uint8_t Get_OPT2_Filtered(void);
uint8_t Get_OPT1_Raw(void);
uint8_t Get_OPT2_Raw(void);

#endif
