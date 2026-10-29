#include "arm_servos.h"

#include "stm32f4xx.h"
#include "delay.h"

void ArmServos_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timebase;
    TIM_OCInitTypeDef output;
    RCC_ClocksTypeDef clocks;
    uint32_t timer_clock;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM5, ENABLE);
    TIM_DeInit(TIM2);
    TIM_DeInit(TIM5);

    RCC_GetClocksFreq(&clocks);
    timer_clock = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) { timer_clock *= 2U; }

    TIM_TimeBaseStructInit(&timebase);
    timebase.TIM_Prescaler = (uint16_t)(timer_clock / 1000000U - 1U);
    timebase.TIM_Period = 20000U - 1U;  /* PA3 lift: 50 Hz. */
    TIM_TimeBaseInit(TIM2, &timebase);
    timebase.TIM_Period = 3000U - 1U;   /* PA1 claw: 333.33 Hz. */
    TIM_TimeBaseInit(TIM5, &timebase);

    TIM_OCStructInit(&output);
    output.TIM_OCMode = TIM_OCMode_PWM1;
    output.TIM_OutputState = TIM_OutputState_Disable;
    output.TIM_Pulse = 0U;
    TIM_OC4Init(TIM2, &output);
    TIM_OC2Init(TIM5, &output);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}

void ArmServos_Lift(uint16_t pulse_us)
{
    TIM_SetCompare4(TIM2, pulse_us);
    TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);
}

void ArmServos_Claw(uint16_t pulse_us)
{
    TIM_SetCompare2(TIM5, pulse_us);
    TIM_CCxCmd(TIM5, TIM_Channel_2, TIM_CCx_Enable);
}

static void ArmServos_MoveSlow(uint16_t *current_us, uint16_t target_us,
                              void (*set_pulse)(uint16_t))
{
    while (*current_us != target_us) {
        if (*current_us < target_us) {
            uint16_t remaining = target_us - *current_us;
            *current_us += remaining < ARM_SERVO_RAMP_STEP_US
                           ? remaining : ARM_SERVO_RAMP_STEP_US;
        } else {
            uint16_t remaining = *current_us - target_us;
            *current_us -= remaining < ARM_SERVO_RAMP_STEP_US
                           ? remaining : ARM_SERVO_RAMP_STEP_US;
        }
        set_pulse(*current_us);
        delay_ms(ARM_SERVO_RAMP_WAIT_MS);
    }
}

void ArmServos_LiftSlow(uint16_t *current_us, uint16_t target_us)
{
    ArmServos_MoveSlow(current_us, target_us, ArmServos_Lift);
}

void ArmServos_ClawSlow(uint16_t *current_us, uint16_t target_us)
{
    ArmServos_MoveSlow(current_us, target_us, ArmServos_Claw);
}
