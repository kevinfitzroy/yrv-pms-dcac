/**********************************************************************************************************
*@file    optocoupler.c
*@date	  2025-06-21
*@brief	  光耦输入滤波模块实现
**********************************************************************************************************/
#include "optocoupler.h"
#include "public.h"

// 光耦滤波相关变量
static uint8_t opt1_raw_state = 0;        // OPT1原始状态
static uint8_t opt1_filtered_state = 0;   // OPT1滤波后状态
static uint8_t opt1_last_state = 0;       // OPT1上一次状态
static uint32_t opt1_change_time = 0;     // OPT1状态变化时间

static uint8_t opt2_raw_state = 0;        // OPT2原始状态
static uint8_t opt2_filtered_state = 0;   // OPT2滤波后状态
static uint8_t opt2_last_state = 0;       // OPT2上一次状态
static uint32_t opt2_change_time = 0;     // OPT2状态变化时间

// 回调函数指针
static OPT_StateChangeCallback opt1_state_callback = NULL;
static OPT_StateChangeCallback opt2_state_callback = NULL;

/*
*@Name		OPT_Init
*@brief		光耦模块初始化
*@prama		opt1_callback: OPT1状态变化回调函数
*@prama		opt2_callback: OPT2状态变化回调函数
*@retval	None
*/
void OPT_Init(OPT_StateChangeCallback opt1_callback, OPT_StateChangeCallback opt2_callback)
{
	// 保存回调函数指针
	opt1_state_callback = opt1_callback;
	opt2_state_callback = opt2_callback;
	
	// 初始化光耦状态
	opt1_raw_state = OPT1;
	opt1_filtered_state = opt1_raw_state;
	opt1_last_state = opt1_raw_state;
	opt1_change_time = 0;
	
	opt2_raw_state = OPT2;
	opt2_filtered_state = opt2_raw_state;
	opt2_last_state = opt2_raw_state;
	opt2_change_time = 0;
}

/*
*@Name		OPT_Filter_Update
*@brief		光耦输入滤波更新函数
*@prama		None
*@retval	None
*/
void OPT_Filter_Update(void)
{
	uint32_t current_time = GetSystemTick();
	
	// 读取OPT1原始状态
	opt1_raw_state = OPT1;
	
	// OPT1滤波处理
	if(opt1_raw_state != opt1_last_state)
	{
		// 状态发生变化，记录变化时间
		opt1_change_time = current_time;
		opt1_last_state = opt1_raw_state;
	}
	else
	{
		// 状态稳定，检查是否超过滤波时间
		if((current_time - opt1_change_time) >= FILTER_TIME_MS)
		{
			// 状态稳定时间超过滤波时间，更新滤波后状态
			if(opt1_filtered_state != opt1_raw_state)
			{
				opt1_filtered_state = opt1_raw_state;
				
				// 调用OPT1状态变化回调函数
				if(opt1_state_callback != NULL)
				{
					opt1_state_callback(opt1_filtered_state);
				}
			}
		}
	}
	
	// 读取OPT2原始状态
	opt2_raw_state = OPT2;
	
	// OPT2滤波处理
	if(opt2_raw_state != opt2_last_state)
	{
		// 状态发生变化，记录变化时间
		opt2_change_time = current_time;
		opt2_last_state = opt2_raw_state;
	}
	else
	{
		// 状态稳定，检查是否超过滤波时间
		if((current_time - opt2_change_time) >= FILTER_TIME_MS)
		{
			// 状态稳定时间超过滤波时间，更新滤波后状态
			if(opt2_filtered_state != opt2_raw_state)
			{
				opt2_filtered_state = opt2_raw_state;
				
				// 调用OPT2状态变化回调函数
				if(opt2_state_callback != NULL)
				{
					opt2_state_callback(opt2_filtered_state);
				}
			}
		}
	}
}

/*
*@Name		Get_OPT1_Filtered
*@brief		获取OPT1滤波后的状态
*@prama		None
*@retval	滤波后的OPT1状态
*/
uint8_t Get_OPT1_Filtered(void)
{
	return opt1_filtered_state;
}

/*
*@Name		Get_OPT2_Filtered
*@brief		获取OPT2滤波后的状态
*@prama		None
*@retval	滤波后的OPT2状态
*/
uint8_t Get_OPT2_Filtered(void)
{
	return opt2_filtered_state;
}

/*
*@Name		Get_OPT1_Raw
*@brief		获取OPT1原始状态
*@prama		None
*@retval	OPT1原始状态
*/
uint8_t Get_OPT1_Raw(void)
{
	return opt1_raw_state;
}

/*
*@Name		Get_OPT2_Raw
*@brief		获取OPT2原始状态
*@prama		None
*@retval	OPT2原始状态
*/
uint8_t Get_OPT2_Raw(void)
{
	return opt2_raw_state;
}
