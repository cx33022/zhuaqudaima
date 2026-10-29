#ifndef __LED_H
#define __LED_H

#include <stdint.h>

void LED_Init(void);
void LED2_Init(void);
void LED_Toggle0(void);
void LED_Toggle1(void);
void LED_Toggle(void);
void LED_Toggle2(void);
void LED_Set2(uint8_t on);

#endif
