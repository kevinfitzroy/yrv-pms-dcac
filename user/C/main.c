/**
 * @file main.c
 * @date 2025-06-21
 * @brief 主程序入口
 * 包含系统初始化、光耦模块初始化和主循环逻辑
 * @author yrv
 * @version V1.0.0
 */
#include "public.h"

uint32_t systemTick = 0;	// 系统时间计数器（毫秒）
uint8_t HandSwitch = 1;		// 手动开关状态（0=接通，1=断开）
uint8_t Emergency_Mode = 0; // 紧急模式标志（0=正常，1=紧急模式）

/**
 * @brief 获取系统时间计数
 * @return 当前系统时间计数（单位：毫秒）
 */
uint32_t GetSystemTick(void)
{
	return systemTick;
}

/*
 *@Name		OPT1_StateChange_Callback
 *@brief		OPT1状态变化回调函数
 *@prama		new_state: 0 表示光耦接通 1 表示光耦断开
 *@retval	None
 */
void OPT1_StateChange_Callback(uint8_t new_state)
{
	// OPT1状态变化时控制LED
	HandSwitch = new_state; // 假设HandSwitch是一个全局变量，用于控制手动开关状态
	LED = !new_state;
}

/*
 *@Name		OPT2_StateChange_Callback
 *@brief		OPT2状态变化回调函数 - 紧急安全保护
 *@prama		new_state: 0 表示光耦接通(紧急状态) 1 表示光耦断开(正常)
 *@retval	None
 */
void OPT2_StateChange_Callback(uint8_t new_state)
{
	if (new_state == 0) // OPT2被触发，进入紧急模式
	{
		Emergency_Mode = 1; // 设置紧急模式标志
		HandSwitch = 1;		// 强制断开手动开关
		RELAY1 = 0;			// 立即关闭继电器1
		RELAY2 = 0;			// 立即关闭继电器2
		relay1_allow = 0;	// 禁止继电器1操作
		relay2_allow = 0;	// 禁止继电器2操作

		// 发送紧急状态CAN报文
		CAN_Send_DCAC_Status(3, 1); // SysStatus=3(EMERGENCY), HandSwitch=1(断开)

		LED = 1; // 点亮LED指示紧急状态
	}
}

/**
 * @brief 更新继电器控制状态
 * @note 根据relay_allow、EnableDCAC状态和紧急模式控制继电器1和继电器2
 */
void Update_Relay_Control(void)
{
	if (Emergency_Mode) // 紧急模式下强制关闭所有继电器
	{
		RELAY1 = 0;
		RELAY2 = 0;
		return;
	}

	RELAY1 = relay1_allow & EnableDCAC; // 控制继电器1
	RELAY2 = relay2_allow & EnableDCAC; // 控制继电器2
}
/**
 * 备用逻辑
 */
void Update_Relay_Control2(void)
{
	if (Emergency_Mode) // 紧急模式下强制关闭继电器
	{
		RELAY1 = 0;
		return;
	}

	RELAY1 = EnablePWM && EnableDCAC; // 控制继电器1
}

void system_init(void)
{
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON); // 打开外部高速晶振HSE
	if (RCC_WaitForHSEStartUp() == SUCCESS)
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1); // 设置AHB时钟
		RCC_PCLK2Config(RCC_HCLK_Div1);	 // 至APB2外设
		RCC_PCLK1Config(RCC_HCLK_Div2);	 // 至通用定时器TIMx和APB1外设，TIMx内部时钟源=72MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
			;
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while (RCC_GetSYSCLKSource() != 0x08)
			;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 打开要使用的外设时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	}
	GPIO_Configuration(); // 端口初始化
	delay_Configuration(72);
	CAN1_Init(); // can通信测试初始化

	USART1_Configuration();							// 串口1初始化
	TIM2_Configuration();							// 定时器2初始化，用于Modbus帧间隔检测
	EN4851 = 1;										// 第一路485芯片使能端拉高设为输出状态
	usart1_send_string("Modbus Slave ID:66 Ready"); // 发送字符串
	usart1_send_one_byte(0x0a);						// 换行显示
	EN4851 = 0;										// 重新配置为接收状态

	IWDG_Init(IWDG_Prescaler_128, 625); // 独立看门狗初始化，超时时间约2s
	LED = 1;							// LED点亮
	delay_ms(500);						// 延时500ms
	LED = 0;
}

int main(void)
{
	system_init();

	// 初始化光耦模块，传入回调函数
	OPT_Init(OPT1_StateChange_Callback, OPT2_StateChange_Callback);

	while (1)
	{
		systemTick++; // 系统时间计数（假设主循环约1ms执行一次）
		delay_ms(1);  // 延时1毫秒，模拟系统时间的增加
		CLRWDT();	  // 喂狗，防止看门狗复位
		Process_CAN_Logs();
		// 紧急模式处理
		if (Emergency_Mode)
		{
			// 在紧急模式下，持续闪烁LED指示危险状态
			if (systemTick % 200 == 0) // 每200ms切换LED状态
			{
				LED = !LED;
			}

			// 发送紧急状态报文
			if (systemTick % 500 == 0) // 每500ms发送紧急状态
			{
				CAN_Send_DCAC_Status(3, HandSwitch); // SysStatus=3(EMERGENCY)
			}

			// 在紧急模式下跳过正常逻辑处理
			continue;
		}

		// 更新光耦滤波状态
		OPT_Filter_Update();

		// 检查接收超时
		if (systemTick % 1000 == 0) // 每秒检查一次
		{
			CAN_Check_Timeout();
			if (Modbus_Check_Timeout())
			{
				Update_Relay_Control();
			}
		}

		// 每100ms发送一次DCAC_Status报文
		if (systemTick % 100 == 0) // 每100ms发送一次
		{
			CAN_Send_DCAC_Status(2, HandSwitch); // SysStatus=2(RUN), HandSwitch为全局变量
		}

		// 主逻辑
		// 检查EnableDCAC状态变化并控制继电器
		// 只要EnableDCAC_Changed标志被设置，就禁止继电器1和继电器2的操作
		if (EnableDCAC_Changed)
		{
			EnableDCAC_Changed = 0;
			// RELAY1 = !!EnableDCAC;
			// RELAY2 = !!EnableDCAC;
			relay2_allow = 0;
			relay1_allow = 0;
			Update_Relay_Control(); // 更新继电器控制状态
		}

		// 备用逻辑
		// if(EnableDCAC_Changed || EnablePWM_Changed)
		// {
		// 	// 如果EnableDCAC或EnablePWM状态发生变化，更新继电器控制
		// 	Update_Relay_Control2();
		// 	EnableDCAC_Changed = 0; // 清除标志
		// 	EnablePWM_Changed = 0; // 清除标志
		// }

		// 处理Modbus帧接收完成
		EN4851 = 0; // 485配置为数据接收状态
		if (modbus_frame_complete)
		{
			modbus_frame_complete = 0;
			receive_flag = 1;
			Receive_Data(); // 处理Modbus数据

			Update_Relay_Control(); // 更新继电器控制状态

			receive_count = 0; // 清除接收计数
		}
	}
}
