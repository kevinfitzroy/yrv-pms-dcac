/******************************************************************************
* @file    can.h
* @version V1.0.0
* @date    2025-06-21
* @brief   can.h
******************************************************************************/
#ifndef __CAN_H
#define __CAN_H

// 函数声明
void CAN1_Init(void);
void CAN_Check_Timeout(void);
void CAN_Send_DCAC_Status(uint8_t SysStatus, uint8_t HandSwitch);
void Process_CAN_Logs(void);

// 外部变量声明（使用extern关键字）
extern uint8_t EnableDCAC;
extern uint8_t EnablePWM;

extern uint32_t lastReceiveTime;
extern uint8_t EnableDCAC_Changed;
extern uint8_t EnablePWM_Changed;
extern uint32_t TIMEOUT_THRESHOLD;

#endif
