/******************************************************************************
 * @file    timer.c
 * @version V1.0.0
 * @date    2025-06-21
 * @brief   定时器配置和中断处理
 ******************************************************************************/
#include "public.h"

uint8_t modbus_frame_complete = 0;

/*
 *@Name		TIM2_Configuration
 *@brief		定时器2配置，用于Modbus RTU帧间隔检测
 *@prama		None
 *@retval	None
 */
void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// 配置定时器2，3.5个字符时间约0.3ms@115200bps
	TIM_TimeBaseStructure.TIM_Period = 300 - 1;   // 0.3ms
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 1MHz时钟
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
 *@Name		TIM2_IRQHandler
 *@brief		定时器2中断处理，检测Modbus帧结束
 *@prama		None
 *@retval	None
 */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		TIM_Cmd(TIM2, DISABLE); // 停止定时器

		if (receive_count > 0)
		{
			modbus_frame_complete = 1; // 标记帧接收完成
		}
	}
}
