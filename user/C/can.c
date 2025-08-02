/******************************************************************************
 * @file    can.c
 * @version V1.0.0
 * @date    2025-06-21
 * @brief   监听0x04080000扩展ID报文，提取EnableDCAC状态
 *          并在3秒内未接收到报文则设置EnableDCAC为0
 *          添加发送DCAC_Status报文的函数，根据报文定义配置各个字段
 * @author  yrv
 ******************************************************************************/
#include "public.h"

// 新增全局变量
uint8_t EnableDCAC = 0;			   // EnableDCAC状态，0=关闭，1=开启
uint8_t EnablePWM = 0;			   // EnablePWM状态，0=使能，1=禁能
uint32_t lastReceiveTime = 0;	   // 上次接收到0x04080000报文的时间
uint8_t EnableDCAC_Changed = 0;	   // EnableDCAC状态变化标志
uint8_t EnablePWM_Changed = 0;	   // EnablePWM状态变化标志
uint32_t TIMEOUT_THRESHOLD = 3000; // 超时阈值，单位为毫秒（3秒）
static uint32_t busRecoverCount = 0;
volatile bool canBusOffLogged = 0; // 中断里置位，主循环里处理
static uint8_t CAN_TIMEOUT_MSG[] = "CAN RX Timeout, EnableDCAC=0";
/*
 *@Name		CAN1_Init
 *@brief		can1通信初始化
 *@prama		None
 *@retval	None
 */
void CAN1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	uint32_t extId; // 将变量声明移到函数开头

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); // CAN1模块时钟使能
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // CANRX接PB8,配置为上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // CANTX接PB9,配置为复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	CAN_DeInit(CAN1);					// 将外设CAN的全部寄存器重设为缺省值
	CAN_StructInit(&CAN_InitStructure); // 把CAN_InitStruct中的每一个参数按缺省值填入

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;		  // 没有使能时间触发模式
	CAN_InitStructure.CAN_ABOM = ENABLE;		  // 没有使能自动离线管理
	CAN_InitStructure.CAN_AWUM = DISABLE;		  // 没有使能自动唤醒模式
	CAN_InitStructure.CAN_NART = DISABLE;		  // 没有使能非自动重传模式
	CAN_InitStructure.CAN_RFLM = DISABLE;		  // 没有使能接收FIFO锁定模式
	CAN_InitStructure.CAN_TXFP = DISABLE;		  // 没有使能发送FIFO优先级
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // CAN设置为正常模式
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;	  // 重新同步跳跃宽度1个时间单位
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;	  // 时间段1为3个时间单位
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;	  // 时间段2为2个时间单位
	CAN_InitStructure.CAN_Prescaler = 24;		  // 时间单位长度为24
	CAN_Init(CAN1, &CAN_InitStructure);			  // 波特率为：72M/2/24(1+3+2)=6/24=0.25 即波特率为250KBPs

	// 1) 接收中断：FIFO0 消息挂号
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 3) 状态/错误中断：Bus-Off, Wake-Up, Error Passive, Error Warning
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 使能中断源
	// 使能接收和状态/错误中断
	CAN_ITConfig(CAN1,
				 CAN_IT_FMP0 |	 // 接收 FIFO0 中断
					 CAN_IT_BOF, // Bus-Off 事件
								 //  CAN_IT_EPV | // Error-Passive
								 //  CAN_IT_EWG | // Error-Warning
								 //  CAN_IT_WKU,  // Wake-Up（AWUM）
				 ENABLE);

	// CAN filter init 过滤器 - 配置为只接收0x04080000扩展ID
	CAN_FilterInitStructure.CAN_FilterNumber = 0;					 // 指定过滤器为0
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;	 // 指定过滤器为标识符列表模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; // 过滤器位宽为32位

	// 扩展ID 0x04080000 的配置
	// 扩展ID需要左移3位，并设置IDE位
	extId = (0x04080000 << 3) | CAN_ID_EXT;
	CAN_FilterInitStructure.CAN_FilterIdHigh = (extId >> 16) & 0xFFFF; // 扩展ID高16位
	CAN_FilterInitStructure.CAN_FilterIdLow = extId & 0xFFFF;		   // 扩展ID低16位

	// 第二个过滤器ID设置为相同值（32位列表模式下有两个ID）
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (extId >> 16) & 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = extId & 0xFFFF;

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;  // 设定了指向过滤器的FIFO为0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; // 使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);
	lastReceiveTime = GetSystemTick();
}
/*
 *@Name		USB_LP_CAN1_RX0_IRQHandler
 *@brief		USB中断和CAN接收中断服务程序，USB跟CAN公用I/O，这里只用到CAN的中断
 *@prama		None
 *@retval	None
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	uint8_t newEnableDCAC, newEnablePWM;

	RxMessage.StdId = 0x00; // 初始化接收结构体变量
	RxMessage.ExtId = 0x00;
	RxMessage.IDE = 0;
	RxMessage.DLC = 0;
	RxMessage.FMI = 0;
	RxMessage.Data[0] = 0x00;
	RxMessage.Data[1] = 0x00;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); // 接收FIFO0中的数据

	// 新增：监听0x04080000扩展ID报文
	if ((RxMessage.ExtId == 0x04080000) && (RxMessage.DLC == 8) && (RxMessage.IDE == CAN_ID_EXT))
	{
		// 更新接收时间
		lastReceiveTime = GetSystemTick(); // 需要实现系统时间获取函数

		// 提取EnableDCAC状态（Data[0]的bit0）
		newEnableDCAC = RxMessage.Data[0] & 0x01;
		newEnablePWM = RxMessage.Data[1] & 0x01;
		// 检查状态是否变化
		if (newEnableDCAC != EnableDCAC)
		{
			EnableDCAC = newEnableDCAC;
			EnableDCAC_Changed = 1; // 设置状态变化标志
		}

		if (newEnablePWM != EnablePWM)
		{
			EnablePWM = newEnablePWM;
			EnablePWM_Changed = 1; // 如果需要，可以设置状态变化标志
		}
	}
	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
}

// --------------------- 状态/错误中断 & 自动恢复 ---------------------
void CAN1_SCE_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1, CAN_IT_BOF))
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
		busRecoverCount++;
		canBusOffLogged = 1; // 标志打一次
	}
}

// 主循环里定期检查并写日志
/**
 * call by main loop
 */
void Process_CAN_Logs(void)
{
	char buf[32];
	int len;
	if (canBusOffLogged)
	{
		canBusOffLogged = 0;

		// 在主上下文安全调用 sprintf + Log_Write
		len = sprintf(buf, "CAN RECOVERED #%lu", busRecoverCount);
		Log_Write(1, (uint8_t *)buf, len);
	}
}
/*
 *@Name		CAN_Check_Timeout
 *@brief		检查接收超时，超过3秒未接收到报文则设置EnableDCAC为0
 *@prama		None
 *@retval	None
 */
void CAN_Check_Timeout(void)
{
	uint32_t currentTime = GetSystemTick();

	// 检查是否超过3秒未接收到报文
	if ((currentTime - lastReceiveTime) > TIMEOUT_THRESHOLD && EnableDCAC != 0)
	{
		EnableDCAC = 0;
		EnableDCAC_Changed = 1; // 设置状态变化标志
		// 记录CAN接收超时日志
		Log_Write(2, CAN_TIMEOUT_MSG, sizeof(CAN_TIMEOUT_MSG) - 1); // 级别2: ERROR
	}
}

/*
 *@Name		CAN_Send_DCAC_Status
 *@brief		发送DCAC_Status报文 (ID=04C80000h)
 *@param		SysStatus: 系统状态 (0-15) RUN 2 ERROR 3
 *@param		HandSwitch: 手动开关状态 (0=接通, 1=断开)
 *@retval	None
 */
void CAN_Send_DCAC_Status(uint8_t SysStatus, uint8_t HandSwitch)
{
	// uint8_t mbox;
	unsigned int i = 0;
	CanTxMsg TxMessage;
	uint8_t temp_const = 80; // 假设温度常量为30,offset 是 -50（单位为摄氏度）

	// 配置CAN报文
	TxMessage.StdId = 0x0000;	  // 标准标识符不使用
	TxMessage.ExtId = 0x04C80000; // 扩展标识符
	TxMessage.IDE = CAN_ID_EXT;	  // 使用扩展标识符
	TxMessage.RTR = CAN_RTR_DATA; // 数据帧
	TxMessage.DLC = 8;			  // 数据长度8字节

	// 清零数据
	for (i = 0; i < 8; i++)
	{
		TxMessage.Data[i] = 0x00;
	}

	// 按照报文定义填充数据
	// SysStatus: unsigned 0,4 (byte0 bit0-3)
	TxMessage.Data[0] |= (SysStatus & 0x0F);

	// HandSwitch: unsigned 4,1 (byte0 bit4)
	TxMessage.Data[0] |= ((HandSwitch & 0x01) << 4);

	// TempModule: unsigned 8,8 (byte1)
	TxMessage.Data[1] = temp_const;

	// TempCapOBG: unsigned 16,8 (byte2)
	TxMessage.Data[2] = temp_const;

	// TempCapOBS: unsigned 24,8 (byte3)
	TxMessage.Data[3] = temp_const;

	// DC_VolMea: unsigned 32,16 (byte4-5)
	TxMessage.Data[4] = RELAY1;
	TxMessage.Data[5] = RELAY2;

	// AC_VolMea: signed 48,16 (byte6-7)
	TxMessage.Data[6] = OPT1;
	TxMessage.Data[7] = OPT2;

	// 发送报文 不怕丢帧 不需要监测 mailbox，或者等待ok
	CAN_Transmit(CAN1, &TxMessage);
	// i = 0;
	// while(i < 0xfff)
	// {
	// 	if((CAN_TransmitStatus(CAN1, mbox) == CANTXOK))
	// 	{
	// 		break;
	// 	}
	// 	i++;
	// }
}
