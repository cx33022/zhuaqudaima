/* USART6 PC6 TX / PC7 RX, 115200 8N1. Standalone PA3/PA1 calibration. */
#ifndef SERVO_CALIBRATION_H
#define SERVO_CALIBRATION_H
#include "stm32f4xx.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>

/* PA3 lift calibration range. PA1 uses the full pulsed range of its 3 ms period. */
#define CAL_MIN_US 500U
#define CAL_MAX_US 2500U
#define CAL_STEP_MIN_US 5U
#define CAL_STEP_MAX_US 100U
#define CAL_STEP_CHANGE_US 5U
static uint16_t cal_pulse[2] = {0, 0};
static uint16_t cal_step_us = 10U;

static void CalSend(const char *s) {
    while (*s) {
        while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART6, (uint8_t)*s++);
    }
}

static void CalStatus(void) {
    char reply[96];
    sprintf(reply, "SETPOINT LIFT(PA3)=%u CLAW(PA1)=%u STEP=%u us; 0=OFF\r\n",
            (unsigned)cal_pulse[0], (unsigned)cal_pulse[1], (unsigned)cal_step_us);
    CalSend(reply);
}

static void CalSetPulse(unsigned channel, int32_t value, const char *command) {
    char reply[64];
    if (channel == 0U && (value < (int32_t)CAL_MIN_US || value > (int32_t)CAL_MAX_US)) {
        CalSend("ERR LIFT RANGE: 500..2500 us\r\n");
        CalStatus();
        return;
    }
    if (channel == 1U && (value < 1 || value >= 3000)) {
        CalSend("ERR CLAW PWM: 1..2999 us at 333.33Hz\r\n");
        CalStatus();
        return;
    }
    if (channel == 0U) {
        TIM_SetCompare4(TIM2, (uint16_t)value);
        TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);
    } else {
        TIM_SetCompare2(TIM5, (uint16_t)value);
        TIM_CCxCmd(TIM5, TIM_Channel_2, TIM_CCx_Enable);
    }
    cal_pulse[channel] = (uint16_t)value;
    sprintf(reply, "OK %s=%ld us\r\n", command, (long)value);
    CalSend(reply);
    CalStatus();
}

static void CalCommand(const char *line) {
    unsigned value = 0U, i, channel;
    int32_t delta;
    char reply[64];
    if (!strcmp(line, "STATUS")) { CalStatus(); return; }
    if (!strcmp(line, "OFF")) {
        TIM_CCxCmd(TIM5, TIM_Channel_2, TIM_CCx_Disable);
        TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Disable);
        cal_pulse[0] = cal_pulse[1] = 0U;
        CalSend("OK OFF BOTH\r\n");
        CalStatus();
        return;
    }
    if (!strcmp(line, "+") || !strcmp(line, "-")) {
        if (line[0] == '+') {
            if (cal_step_us < CAL_STEP_MAX_US) { cal_step_us += CAL_STEP_CHANGE_US; }
        } else if (cal_step_us > CAL_STEP_MIN_US) {
            cal_step_us -= CAL_STEP_CHANGE_US;
        }
        sprintf(reply, "OK STEP=%u us\r\n", (unsigned)cal_step_us);
        CalSend(reply);
        CalStatus();
        return;
    }
    if (line[1] == '\0' && line[0] >= '1' && line[0] <= '4') {
        channel = (line[0] <= '2') ? 0U : 1U;
        if (cal_pulse[channel] == 0U) {
            CalSend(channel == 0U ? "ERR LIFT OFF; set 1:us first\r\n"
                                  : "ERR CLAW OFF; set 2:us first\r\n");
            CalStatus();
            return;
        }
        /* 1: PA3 pulse + (previously up); 2: PA3 pulse - (down).
         * 3: PA1 pulse - (previously open); 4: PA1 pulse + (close). */
        delta = (line[0] == '1' || line[0] == '4')
                ? (int32_t)cal_step_us : -(int32_t)cal_step_us;
        CalSetPulse(channel, (int32_t)cal_pulse[channel] + delta, line);
        return;
    }
    if ((line[0] != '1' && line[0] != '2') || line[1] != ':' || !line[2]) {
        CalSend("ERR FORMAT: 1/2/3/4/+/-/1:us/2:us/STATUS/OFF\r\n");
        CalStatus();
        return;
    }
    for (i = 2; line[i]; ++i) {
        if (i > 5 || line[i] < '0' || line[i] > '9') {
            CalSend("ERR VALUE\r\n");
            CalStatus();
            return;
        }
        value = value * 10U + (unsigned)(line[i] - '0');
    }
    channel = (unsigned)(line[0] - '1');
    CalSetPulse(channel, (int32_t)value, line);
}

static void ServoCalibrationRun(void) {
    GPIO_InitTypeDef g;
    USART_InitTypeDef u;
    TIM_TimeBaseInitTypeDef t;
    TIM_OCInitTypeDef oc;
    RCC_ClocksTypeDef clocks;
    uint32_t timer_clock;
    char line[24];
    unsigned length = 0, discard = 0, quiet_ticks = 0;
    uint32_t rx_errors = 0;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM5, ENABLE);
    GPIO_StructInit(&g);
    g.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &g);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
    USART_StructInit(&u);
    u.USART_BaudRate = 115200;
    USART_Init(USART6, &u);
    USART_Cmd(USART6, ENABLE);

    TIM_DeInit(TIM2);
    TIM_DeInit(TIM5);
    RCC_GetClocksFreq(&clocks);
    timer_clock = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) timer_clock *= 2;
    TIM_TimeBaseStructInit(&t);
    t.TIM_Prescaler = (uint16_t)(timer_clock / 1000000U - 1U);
    t.TIM_Period = 20000U - 1U;
    TIM_TimeBaseInit(TIM2, &t);
    /* PA1 claw: 3 ms period, confirmed as 333.33 Hz on the signal line. */
    t.TIM_Period = 3000U - 1U;
    TIM_TimeBaseInit(TIM5, &t);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 0;
    TIM_OC2Init(TIM5, &oc);
    TIM_OC4Init(TIM2, &oc);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    g.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
    g.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &g);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
    CalSend("SERVO CAL USART6 READY; PC6=TX PC7=RX; PA3=50Hz PA1=333.33Hz\r\n");
    CalSend("No preset positions; PWM stays OFF until a pulse is entered.\r\n");
    CalSend("Seed: 1:us=PA3 2:us=PA1; then 1/2=PA3 +/- 3/4=PA1 -/+\r\n");
    CalSend("+/- changes step by 5us; STATUS OFF; CR/LF optional\r\n");
    CalStatus();
    while (1) {
        uint32_t status = USART6->SR;
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
            volatile uint32_t ignored = USART6->DR;
            (void)ignored;
            length = 0; discard = 1; quiet_ticks = 0;
            /* Drain the damaged line before printing; TX is blocking. */
            rx_errors |= status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE);
        } else if (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) != RESET) {
            uint8_t b = (uint8_t)USART_ReceiveData(USART6);
            quiet_ticks = 0;
            if (b == '\r' || b == '\n') {
                if (rx_errors) {
                    char reply[64];
                    sprintf(reply, "ERR UART flags=%lu (ORE=8 FE=2 NE=4); resend\r\n",
                            (unsigned long)rx_errors);
                    CalSend(reply);
                    rx_errors = 0;
                } else if (!discard && length) {
                    line[length] = 0;
                    CalCommand(line);
                }
                length = 0; discard = 0;
            } else if (!discard) {
                if (b < 32 || b > 126 || length >= sizeof(line)-1) {
                    length = 0; discard = 1;
                    CalSend("ERR LINE: ASCII only, max 23 chars\r\n");
                } else line[length++] = (char)b;
            }
        } else {
            /* 115200 8N1: one byte every 86.8 us. Never sleep 1 ms here. */
            delay_us(10);
            /* Serial assistants may send text without CR/LF. Execute the
             * buffered command after roughly 100 ms of receive silence. */
            if (++quiet_ticks >= 10000) {
                if (rx_errors) {
                    char reply[64];
                    sprintf(reply, "ERR UART flags=%lu (ORE=8 FE=2 NE=4); resend\r\n",
                            (unsigned long)rx_errors);
                    CalSend(reply);
                } else if (!discard && length) {
                    line[length] = 0;
                    CalCommand(line);
                }
                length = 0; discard = 0; quiet_ticks = 0; rx_errors = 0;
            }
        }
    }
}
#endif
