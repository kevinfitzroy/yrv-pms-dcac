/**********************************************************************************************************
*@file    wdt.h
*@date	  05/02/2023
*@brief	  None
**********************************************************************************************************/
#ifndef __WDT_H
#define __WDT_H
void IWDG_Init(unsigned char prer,unsigned short int rlr);
void CLRWDT(void);
#endif
