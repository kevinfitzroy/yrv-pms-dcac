/******************************************************************************
 * @file    usart.c
 * @version V1.0.0
 * @date    2025-06-21
 * @author  yrv
 * @brief   usart.c
 ******************************************************************************/
#include "public.h"

unsigned char receive_count;
unsigned char receive[MODBUS_BUFFER_SIZE];
unsigned char receive_flag;

// Modbus相关变量
uint8_t relay1_allow = 0;		 // RELAY1允许导通标志
uint8_t relay2_allow = 0;		 // RELAY2允许导通标志
uint32_t relay1_last_update = 0; // RELAY1最后更新时间
uint32_t relay2_last_update = 0; // RELAY2最后更新时间

// CRC16计算函数
uint16_t CRC16(uint8_t *data, uint8_t length)
{
	uint16_t crc = 0xFFFF;
	uint8_t i, j;

	for (i = 0; i < length; i++)
	{
		crc ^= data[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 0x0001)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc = crc >> 1;
		}
	}
	return crc;
}

// Modbus错误响应
void Modbus_Send_Error(uint8_t function_code, uint8_t error_code)
{
	uint8_t response[5];
	uint16_t crc;
	int i;

	response[0] = MODBUS_SLAVE_ID;
	response[1] = function_code | 0x80; // 错误标志
	response[2] = error_code;

	crc = CRC16(response, 3);
	response[3] = crc & 0xFF;
	response[4] = (crc >> 8) & 0xFF;

	EN4851 = 1; // 配置为发送状态
	for (i = 0; i < 5; i++)
	{
		usart1_send_one_byte(response[i]);
	}
	EN4851 = 0; // 重新配置为接收状态
}

// Modbus写单个线圈响应
void Modbus_Write_Single_Coil_Response(uint16_t address, uint16_t value)
{
	uint8_t response[8];
	uint16_t crc;
	int i;

	response[0] = MODBUS_SLAVE_ID;
	response[1] = 0x05; // 写单个线圈功能码
	response[2] = (address >> 8) & 0xFF;
	response[3] = address & 0xFF;
	response[4] = (value >> 8) & 0xFF;
	response[5] = value & 0xFF;

	crc = CRC16(response, 6);
	response[6] = crc & 0xFF;
	response[7] = (crc >> 8) & 0xFF;

	EN4851 = 1; // 配置为发送状态
	for (i = 0; i < 8; i++)
	{
		usart1_send_one_byte(response[i]);
	}
	EN4851 = 0; // 重新配置为接收状态
}

// 日志缓冲区
static LogEntry_t log_buffer[LOG_BUFFER_SIZE];
static uint8_t log_head = 0;  // 写入位置
static uint8_t log_count = 0; // 当前日志数量

// 写入日志
void Log_Write(uint8_t level, const uint8_t *data, uint8_t length)
{
	uint8_t i;
	if (length > LOG_DATA_SIZE)
	{
		length = LOG_DATA_SIZE; // 限制长度
	}

	// 写入新日志
	log_buffer[log_head].timestamp = GetSystemTick();
	log_buffer[log_head].level = level;
	log_buffer[log_head].length = length;

	// 复制日志数据
	for (i = 0; i < length; i++)
	{
		log_buffer[log_head].data[i] = data[i];
	}

	// 更新指针和计数
	log_head = (log_head + 1) % LOG_BUFFER_SIZE;
	if (log_count < LOG_BUFFER_SIZE)
	{
		log_count++; // 只在缓冲区未满时增长
	}
	// 缓冲区满后，log_count 保持为 LOG_BUFFER_SIZE
}

// Modbus读日志响应
void Modbus_Read_Logs_Response(uint16_t start_index, uint16_t count)
{
	uint8_t response[256]; // 足够大的响应缓冲区
	uint16_t response_len = 0;
	uint16_t crc;
	uint8_t actual_count = 0;
	uint8_t read_index;
	int i;
	uint8_t available_logs;
	uint8_t j, k;
	// 计算当前可用的日志数量
	available_logs = (log_count < LOG_BUFFER_SIZE) ? log_count : LOG_BUFFER_SIZE;

	// 计算实际可读取的日志数量
	if (start_index >= available_logs)
	{
		actual_count = 0;
	}
	else
	{
		actual_count = (count > (available_logs - start_index)) ? (available_logs - start_index) : count;
	}

	// 构建响应头
	response[0] = MODBUS_SLAVE_ID;
	response[1] = 0x03;								   // 读保持寄存器功能码
	response[2] = actual_count * (sizeof(LogEntry_t)); // 字节数
	response_len = 3;

	// 添加日志数据
	for (j = 0; j < actual_count; j++)
	{
		// 计算实际读取位置 (从最老的日志开始)
		if (log_count < LOG_BUFFER_SIZE)
		{
			read_index = start_index + j;
		}
		else
		{
			read_index = (log_head + start_index + j) % LOG_BUFFER_SIZE;
		}

		// 复制时间戳 (4字节)
		response[response_len++] = (log_buffer[read_index].timestamp >> 24) & 0xFF;
		response[response_len++] = (log_buffer[read_index].timestamp >> 16) & 0xFF;
		response[response_len++] = (log_buffer[read_index].timestamp >> 8) & 0xFF;
		response[response_len++] = log_buffer[read_index].timestamp & 0xFF;

		// 复制级别和长度
		response[response_len++] = log_buffer[read_index].level;
		response[response_len++] = log_buffer[read_index].length;

		// 复制日志数据
		for (k = 0; k < log_buffer[read_index].length && k < LOG_DATA_SIZE; k++)
		{
			response[response_len++] = log_buffer[read_index].data[k];
		}

		// 如果数据不足32字节，用0填充
		for (k = log_buffer[read_index].length; k < LOG_DATA_SIZE; k++)
		{
			response[response_len++] = 0;
		}
	}

	// 添加CRC
	crc = CRC16(response, response_len);
	response[response_len++] = crc & 0xFF;
	response[response_len++] = (crc >> 8) & 0xFF;

	// 发送响应
	EN4851 = 1; // 配置为发送状态
	for (i = 0; i < response_len; i++)
	{
		usart1_send_one_byte(response[i]);
	}
	EN4851 = 0; // 重新配置为接收状态
}

// Modbus数据处理
void Modbus_Process_Data(void)
{
	uint16_t crc_received, crc_calculated;
	uint8_t function_code;
	uint16_t address, value;
	uint16_t start_addr, quantity;
	extern uint32_t systemTick;

	// 检查CRC
	crc_received = (receive[receive_count - 1] << 8) | receive[receive_count - 2];
	crc_calculated = CRC16(receive, receive_count - 2);

	if (crc_received != crc_calculated)
	{
		// Modbus_Send_Error(0x05, 0x05);//TODO
		return; // CRC错误，忽略数据
	}

	// 检查从机地址
	if (receive[0] != MODBUS_SLAVE_ID)
	{
		// Modbus_Send_Error(0x05, 0x04);//TODO
		return; // 不是发给本机的
	}

	function_code = receive[1];

	switch (function_code)
	{
	case 0x03: // 读保持寄存器 (用于读取日志)
		if (receive_count != 8)
		{
			Modbus_Send_Error(0x03, 0x03); // 非法数据值
			return;
		}

		start_addr = (receive[2] << 8) | receive[3];
		quantity = (receive[4] << 8) | receive[5];

		if (start_addr >= 0x1000 && start_addr < 0x1100)
		{													// 日志查询地址范围
			uint16_t log_start_index = start_addr - 0x1000; // 计算日志起始索引

			if (quantity > 6)
			{								   // 限制一次最多读取10条日志
				Modbus_Send_Error(0x03, 0x03); // 非法数据值
				return;
			}

			Modbus_Read_Logs_Response(log_start_index, quantity);
		}
		else
		{
			Modbus_Send_Error(0x03, 0x02); // 非法数据地址
		}
		break;

	case 0x05: // 写单个线圈
		if (receive_count != 8)
		{
			Modbus_Send_Error(0x05, 0x03); // 非法数据值
			return;
		}

		address = (receive[2] << 8) | receive[3];
		value = (receive[4] << 8) | receive[5];

		if (address == 0x0000) // RELAY1控制地址
		{
			relay1_allow = (value == 0xFF00) ? 1 : 0;
			relay1_last_update = systemTick;
			// RELAY1 = relay1_allow;
			Modbus_Write_Single_Coil_Response(address, value);
		}
		else if (address == 0x0001) // RELAY2控制地址
		{
			relay2_allow = (value == 0xFF00) ? 1 : 0;
			relay2_last_update = systemTick;
			// RELAY2 = relay2_allow;
			Modbus_Write_Single_Coil_Response(address, value);
		}
		else
		{
			Modbus_Send_Error(0x05, 0x02); // 非法数据地址
		}
		break;

	default:
		Modbus_Send_Error(function_code, 0x01); // 非法功能码
		break;
	}
}

// 检查继电器超时
/**
 * @brief 检查继电器超时
 * @return 如果有继电器超时，则返回1，否则返回0
 */
bool Modbus_Check_Timeout(void)
{
	extern uint32_t systemTick;

	// 检查RELAY1超时
	if (relay1_allow && (systemTick - relay1_last_update) > MODBUS_TIMEOUT_MS)
	{
		relay1_allow = 0;
		// RELAY1 = 0; // 切断继电器
	}

	// 检查RELAY2超时
	if (relay2_allow && (systemTick - relay2_last_update) > MODBUS_TIMEOUT_MS)
	{
		relay2_allow = 0;
		// RELAY2 = 0; // 切断继电器
	}

	return !(relay1_allow && relay2_allow);
}

/*
 *@Name		串口1配置
 *@brief		96n81
 *@prama		None
 *@retval	None
 */
void USART1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // PA9为USART1的数据发送端，设为复用推挽输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // PA10为USART1的数据接收端，设为浮空输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200; // 修改为115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;					// 发送接收使能
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开接收中断
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC); // 清除串口1中断标志位

	// 配置NVIC中断控制器
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; // 指定USART1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能中断
	NVIC_Init(&NVIC_InitStructure);
	receive_count = 0;
}
/*
 *@Name		usart1_send_one_byte
 *@brief		串口1发送字节数据
 *@prama		None
 *@retval	None
 */
void usart1_send_one_byte(uchar data)
{
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_SendData(USART1, data);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		; // 等待发送完成
}
/*
 *@Name		usart1_send_string
 *@brief		串口1发送字符串数据
 *@prama		None
 *@retval	None
 */
void usart1_send_string(const char *buf)
{
	while (*buf != '\0')
	{
		usart1_send_one_byte(*buf);
		buf++;
	}
}
/*
 *@Name		串口模式下的接收数据处理
 *@brief		Modbus RTU协议处理
 *@prama		None
 *@retval	None
 */
void Receive_Data(void)
{
	receive_flag = 0; // 清除数据接收成功标志位

	// 处理Modbus数据
	if (receive_count >= 4) // Modbus最小帧长度
	{
		Modbus_Process_Data();
	}
}
/*
 *@Name		USART1_IRQHandler
 *@brief		串口中断处理
 *@prama		None
 *@retval	None
 */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_FLAG_ORE) != RESET) // 清除串口溢出错误
	{
		USART_ReceiveData(USART1);
		USART_ClearITPendingBit(USART1, USART_FLAG_ORE);
	}
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		if (receive_count < MODBUS_BUFFER_SIZE)
		{
			receive[receive_count] = USART_ReceiveData(USART1);
			receive_count++;

			// 每接收到一个字节，重新启动定时器
			TIM_SetCounter(TIM2, 0); // 重置计数器
			TIM_Cmd(TIM2, ENABLE);	 // 启动定时器
		}
		else
		{
			// 缓冲区已满，丢弃数据并重置
			USART_ReceiveData(USART1); // 读取数据但丢弃
			receive_count = 0;		   // 重置计数器
			TIM_Cmd(TIM2, DISABLE);	   // 停止定时器
		}
	}
}
