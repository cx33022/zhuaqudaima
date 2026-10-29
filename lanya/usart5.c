#include "usart5.h"
#include "hmi.h"
#include "LED.h"
#include "Serial.h"
#include "swith.h"
#include "LED.h"
#include "delay.h"

void uart5_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //根据芯片情况选择端口
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //使能UART5时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);	 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 

	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure);      
	
	USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
	USART_Cmd(UART5, ENABLE);                     
}

/* Poll for an unframed byte. Return 0 if no byte is available. */
uint8_t UART5_TryReadByte(uint8_t *value)
{
    if (value == 0 || USART_GetFlagStatus(UART5, USART_FLAG_RXNE) == RESET) { return 0U; }
    *value = (uint8_t)USART_ReceiveData(UART5);
    return 1U;
}

void UART5_SendString(char *str) 
{
	while (*str != '\0') {
		while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
		USART_SendData(UART5, (uint8_t)*str);
		str++;
	}
}

void UART5_SendArray(uint8_t *data, uint16_t len)
{
    uint16_t i;
    for(i = 0; i < len; i++)
    {
        // 等待发送缓冲区为空
        while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
        // 发送一个字节
        USART_SendData(UART5, data[i]);
    }
    // 等待所有数据发送完成
    while (USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);
}

void find_congji()
{
	UART5_SendString("AT+NAME=STM32_Master\r\n");//设置蓝牙模块名称
	delay_ms(1000);
	// 搜索设备 
	UART5_SendString("AT+PSWD=1234\r\n");//两个要对应的pin值
	delay_ms(1000);
	UART5_SendString("AT+ROLE=1\r\n");//设置为主机模式（主动连接别人，不能被主动连接）
	delay_ms(1000);	
	UART5_SendString("AT+BIND=98:DA:20:07:6D:F5\r\n");//写上从机的蓝牙地址
}


void fasong_task_code(void)
{
	Serial_SendByte(0x72);
	uint8_t task_code[7];
//	task_code[3] = 0x2B;
	delay_ms1(500);
//	while(swith_out() == 1){		
//		LED_Toggle2();
//		delay_ms1(50);
//	}
	task_code[0] = 0x31;
	task_code[1] = 0x32;
	task_code[2] = 0x33;
	task_code[3] = 0x2B;
	task_code[4] = 0x33;
	task_code[5] = 0x32;
	task_code[6] = 0x31;
	Serial_SendArray(task_code,7);
}
