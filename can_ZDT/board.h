#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f4xx.h"

/* 0: external CAN bus; 1: MCU internal loopback diagnostic. */
#define CAN_INTERNAL_LOOPBACK_TEST 0

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

void nvic_init(void);
void clock_init(void);
void usart_init(void);
extern volatile uint8_t board_can_init_ok;
void board_init(void);

#endif
