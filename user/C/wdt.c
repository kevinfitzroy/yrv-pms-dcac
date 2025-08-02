/**********************************************************************************************************
 *@file    wdt.c
 *@date	  05/02/2023
 *@brief	  看门狗相关函数
 **********************************************************************************************************/
#include "public.h"
/*
 *@Name		IWDG_Init看门狗初始化程序
 *@brief		None
 *@prama		None
 *@retval	None
 */
void IWDG_Init(unsigned char prer, unsigned short int rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对寄存器IWDG_PR和IWDG_RLR的写操作，即取消写保护
	IWDG_SetPrescaler(prer);					  // 设置IWDG预分频值
	IWDG_SetReload(rlr);						  // 设置IWDG重装载值
	IWDG_ReloadCounter();						  // 按照IWDG重装载寄存器的值重装载IWDG计数器
	IWDG_Enable();								  // 使能IWDG
}
/*
 *@Name		喂狗程序
 *@brief		None
 *@prama		None
 *@retval	None
 */
void CLRWDT(void)
{
	IWDG_ReloadCounter(); // 重装载初值
}
