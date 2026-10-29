#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx.h"

/* 补光灯硬件：PA6 = TIM3_CH1，PWM 频率 84MHz / 84 / 2000 = 500Hz。
 * 亮度靠占空比调节：CCR 0~2000 对应 0~100%，最细一级为 0.05%。
 * LIGHT_DEFAULT_PERCENT 是上电默认亮度，0 = 上电不亮。 */
#define LIGHT_PWM_PERIOD        2000U
#define LIGHT_DEFAULT_PERCENT      0U

void Timer_Init(void);

void Light_Init(void);                    /* 配置 PA6/TIM3_CH1，并设为默认亮度 */
void Light_SetPercent(uint8_t percent);   /* 0~100，超过 100 按 100 处理 */
void Light_SetCcr(uint16_t ccr);          /* 0~2000，最细粒度 */
uint8_t Light_GetPercent(void);           /* 返回上次设置的百分比 */

#endif
