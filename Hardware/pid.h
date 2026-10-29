#ifndef __PID_H
#define __PID_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
#include "LED.h"
#include "Emm_V5.h"

typedef struct
{
   	float kp, ki, kd; //三个系数
    float error, lastError; //误差、上次误差
    float integral, maxIntegral; //积分、积分限幅
    float output, maxOutput; //输出、输出限幅
}PID;

extern volatile int flag;
extern PID mypid;
extern uint8_t dianji_move_flag;
extern uint8_t PID_control_flag;
extern float fAcc[3], fGyro[3], fAngle[3];
extern volatile uint32_t pid_tick_count;
extern volatile uint32_t pid_yaw_value_changed_count;
extern volatile float pid_rate_deg_s;
void PID_DIL(int v_x,int v_y,int w,uint32_t time,int target);

void PIDInit(void);
uint8_t PID_move(int v_x,int v_y,int w,uint32_t time,float target,int pid_choose);
uint8_t PID_turn_stable(float target,uint32_t timeout_ms);
/* 转到目标航向并稳定：先 PID 转到位，停一下，再低速 PID_turn_stable 做末段修正。
   路线里的 ChassisRoute_TurnTo 和横移后的航向拉回都用它。 */
void car_turn_to_stable(float yaw_deg);
/* 横移的航向稳定：横移是位置模式，过程中没有航向闭环，
   用 car_yaw_mark 记下横移前的航向，走完由 car_yaw_hold 拉回来。 */
float car_yaw_mark(void);
void car_yaw_hold(float yaw0);
//单位是mm
void car_move_distance(float x,float y,int a_in,int speed_in);
void PID_car_move_distance(float x,float y,float angel,float speed);

#endif
