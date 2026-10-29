#include "stm32f4xx.h"                  // Device header
#include "yyb_move.h"
#include "OLED.h"
#include "delay.h"
#include <math.h>
#include "swith.h"
#include "wit_c_sdk.h"
#include "PWM.h"
#include <stdio.h>
#include <math.h>
#include "LED.h"
#include "imu.h"

#define pi  3.1415926/180

float feedbackValue = 0; //这里获取到被控对象的反馈值
float feedbackValue_pre = 0;
float targetValue = 0; //这里获取到目标值
float fAcc[3], fGyro[3], fAngle[3];
int vx,vy,omiga;
volatile int flag; /* TIM4 中断写入，主循环等待读取 */
uint32_t delay_time = 0;
uint32_t real_time = 0;
uint8_t dianji_move_flag = 0;
uint8_t PID_control_flag = 0;
uint8_t circle = 0;
float same_time = 0;   //测量PID计算中角度坐标的信息是否重复，也即PID计算的内容是否有效
volatile uint32_t pid_tick_count = 0U;
volatile uint32_t pid_yaw_value_changed_count = 0U; /* 仅表示角度数值变化，不等同于IMU新帧计数 */
volatile float pid_rate_deg_s = 0.0f;

/* 转向末段：不用积分，克服低速死区，并在角度连续稳定后提前结束。 */
#define TURN_STABLE_TOLERANCE_DEG  0.5f
#define TURN_STABLE_SAMPLE_COUNT   5U
#define TURN_STABLE_MIN_SPEED      5.0f
#define TURN_STABLE_MAX_SPEED      20.0f
#define TURN_STABLE_MAX_RATE_DEG_S 2.0f
#define PID_GYRO_Z_SIGN            1.0f /* 若实测角度增大时GZ为负，只需改成-1.0f */
static volatile uint8_t turn_stable_mode = 0U;
static volatile uint8_t turn_stable_count = 0U;

/* “转到目标航向并稳定”的三段时序，数值与原来 ChassisRoute_TurnTo 里的一致 */
#define CAR_TURN_MAX_SPEED         100.0f  /* 转向第一段的最高转速(RPM)：原来的限速是 180~230，
                                              而 kp=2 时 180° 误差能算出 360RPM，等于全程满转，
                                              所以这里单独压一下。嫌快就调小，嫌慢就调大 */
#define CAR_TURN_REACHED_DEG       2.0f    /* 第一段转进这个范围就交给末段精修，不再空等 */
#define CAR_TURN_REACHED_COUNT     2U      /* 连续 2 次(100ms)都在范围内才算到位，防穿靶 */
#define CAR_TURN_TIME_MS           4000U   /* 第一段的超时；转到位会提前结束，通常走不到这里 */
#define CAR_TURN_SETTLE_MS         200U    /* 两段之间停车稳一下，等车体和陀螺仪静下来 */
#define CAR_TURN_STABLE_TIMEOUT_MS 2000U   /* 末段 PID_turn_stable 的超时 */

static volatile uint8_t turn_approach_mode = 0U;  /* 1 = 第一段转到 ±CAR_TURN_REACHED_DEG 就结束 */
static volatile uint8_t turn_approach_count = 0U;

/*
	横移的航向稳定：横移走位置模式（四轮同步发脉冲），过程中完全没有航向闭环，
	横移越大方向偏得越多。这里沿用转向后那套 car_turn_to_stable：
	横移前记下航向，横移走完后把车头拉回横移开始时的朝向。

	CAR_YAW_SETTLE_MS：car_move_delay 是估算值，测航向前先留点余量把车停稳，
	别在位置段还没走完时就插进速度指令（那样会打断这一程横移），
	顺带也能保证收到一帧新的 HWT101 航向。
*/
#define CAR_YAW_HOLD_ENABLE     1       /* 0 = 关闭横移后的航向稳定 */
#define CAR_YAW_HOLD_MIN_MM     1.0f    /* 横移距离小于它就不做，没动就不用修正 */
#define CAR_YAW_HOLD_TOL_DEG    0.5f    /* 已经在这个范围内就不转，省掉一整轮稳定 */
#define CAR_YAW_HOLD_MAX_DEG    30.0f   /* 误差大得离谱多半是航向数据异常，宁可不转也别乱转 */
#define CAR_YAW_HOLD_TIME_MS    1000U   /* 修正第一段的超时；转到 ±2° 会提前结束，一般走不到这里 */
#define CAR_YAW_SETTLE_MS       250U    /* 横移走完先等车停稳，再测航向 */

typedef struct
{
   	float kp, ki, kd; //三个系数
    float error, lastError; //误差、上次误差
    float integral, maxIntegral; //积分、积分限幅
    float output, maxOutput; //输出、输出限幅
}PID;
PID mypid = {0}; //创建一个PID结构体变量

void Time_Init();
void Clear_Init();

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
}
 

/**
  * @brief  进行一次pid计算
  *         参数为(pid结构体,目标值,反馈值)
  *                    
  * @param  pid
  * @param  reference: target_of_value 
  * @param  feedback 
  * @retval None
  */
void PID_Calc(PID *pid, float reference, float feedback)
{
 	//更新数据
    pid->lastError = pid->error; //将旧error存起来
    //计算新误差
    if(reference - feedback < -180){
		pid->error = reference - feedback + 360; //计算新error
	}else if(reference - feedback > 180){
		pid->error = reference - feedback - 360;
	}else{
		pid->error = reference - feedback;
	}	
    //微分直接使用陀螺仪角速度，避免误差差分放大量化噪声和重复角度帧
    float dout = -pid_rate_deg_s * pid->kd;
    //计算比例
    float pout = pid->error * pid->kp;
    //计算积分
    pid->integral += pid->error * pid->ki;
    //积分限幅
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
    //计算输出
    pid->output = pout+dout + pid->integral;
    //输出限幅
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}
 
void PIDInit(void)
{
	Time_Init();
    PID_Init(&mypid, 1.5, 0.001, 0.1, 7, 180); //D项改用deg/s后，kd按50ms周期缩放
}

void PID_DIL(int v_x,int v_y,int w,uint32_t time,float target)
{   
	vx = v_x;
	vy = v_y;
	omiga = w;
	delay_time = time;
	real_time = 0;
	targetValue = target;
}

uint8_t PID_move(int v_x,int v_y,int w,uint32_t time,float target,int pid_choose)
{   
	TIM_Cmd(TIM4, ENABLE); // 使能TIM4，定时器开始运行
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	flag = 0; //运动标志位归零
	change_A(230);//设置PID move 的加速度为 230rpm
	if(pid_choose == 0){
		mypid.integral = 0.0f;
		mypid.kp = 2;
		mypid.ki = 0.00;
		mypid.kd = 0.04;
	}else if(pid_choose == 1){
		mypid.integral = 0.0f;
		mypid.kp = 3.3;
		mypid.ki = 0.00;
		mypid.kd = 0.09;
		mypid.maxOutput = 30;
	}else if(pid_choose == 2){
		mypid.integral = 0.0f;
		mypid.kp = 5;
		mypid.ki = 0.00;
		mypid.kd = 0.25;
	}else if(pid_choose == 3){
		mypid.kp = 2;
		mypid.ki = 0.05;
		mypid.kd = 0.02;
	}else if(pid_choose == 4){
		mypid.kp = 2;
		mypid.ki = 0.07;
		mypid.kd = 0.02;
	}
	if(target > 180){
		target = target -360;
	}else if(target < -180){
		target = target + 360;
	}
	PID_DIL(v_x,v_y,w,time,target);
	while(flag == 0){};flag = 0;
	mypid.maxOutput = 230;
	TIM_Cmd(TIM4, DISABLE); 
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	delay_ms1(100);//等待定时器完全关闭
	return 1;
}

uint8_t PID_turn_stable(float target,uint32_t timeout_ms)
{
	uint8_t result;

	turn_stable_count = 0U;
	turn_stable_mode = 1U;
	mypid.integral = 0.0f;
	mypid.error = 0.0f;
	mypid.lastError = 0.0f;
	mypid.maxOutput = TURN_STABLE_MAX_SPEED;
	result = PID_move(0, 0, 0, timeout_ms, target, 0);
	turn_stable_mode = 0U;
	turn_stable_count = 0U;
	mypid.integral = 0.0f;
	return result;
}

/* 把 target - current 折算到 ±180° 以内，避免过零时算出 ±359° 的假误差 */
static float car_yaw_error(float target, float current)
{
	float error = target - current;

	while (error > 180.0f) { error -= 360.0f; }
	while (error < -180.0f) { error += 360.0f; }
	return error;
}

/*
	转到目标航向并稳定，就是原来路线里 ChassisRoute_TurnTo 的效果：
	第一段用正常航向 PID 转到位（限速 CAR_TURN_MAX_SPEED，转到 ±2° 就提前结束），
	停一下等车体和陀螺仪稳住，第二段用低速 PID_turn_stable 做末段修正。
	路线转向和横移后的航向拉回都调这一支，两边不会再越改越不一样。
*/
static void car_turn_to_stable_time(float yaw_deg, uint32_t time_ms)
{
	mypid.integral = 0.0f;
	mypid.maxOutput = CAR_TURN_MAX_SPEED;   /* 单独限速，别一上来就 200 多转 */
	turn_approach_count = 0U;
	turn_approach_mode = 1U;
	PID_move(0, 0, 0, time_ms, yaw_deg, 0); /* 转到 ±CAR_TURN_REACHED_DEG 就提前返回，time_ms 只是超时 */
	turn_approach_mode = 0U;
	turn_approach_count = 0U;
	car_move(0, 0, 0);                      /* 明确发一次零速，别让上一段的速度指令拖着 */
	delay_ms1(CAR_TURN_SETTLE_MS);          /* 等车体和陀螺仪停稳后再做末段修正 */
	mypid.integral = 0.0f;
	PID_turn_stable(yaw_deg, CAR_TURN_STABLE_TIMEOUT_MS);
	mypid.integral = 0.0f;
}

/* 路线转向用：第一段最多转 CAR_TURN_TIME_MS，转到位会提前结束 */
void car_turn_to_stable(float yaw_deg)
{
	car_turn_to_stable_time(yaw_deg, CAR_TURN_TIME_MS);
}

/*
	横移前取一次航向。
	PID_move 结束时会关掉 USART2 的接收中断（IMU 数据随之停更新），
	所以这里先重新打开，再等一帧新数据，避免拿到上一次 PID 留下的旧航向。
*/
float car_yaw_mark(void)
{
#if CAR_YAW_HOLD_ENABLE
	if (!imu_angle_ready()) { return 0.0f; }   /* 没有 HWT101 就不要动航向 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	delay_ms1(CAR_YAW_SETTLE_MS);
	imu_scan(fAcc, fGyro, fAngle);
	return fAngle[2];
#else
	return 0.0f;
#endif
}

/*
	横移结束后把航向拉回 yaw0，用的就是转向后那套 car_turn_to_stable，
	所以手感和转向后的稳定完全一致。
	误差已经在容差内就不动（省掉整整一轮 1 s + 末段修正）；
	误差超过 CAR_YAW_HOLD_MAX_DEG 也不动：一次横移不可能偏这么多，
	只可能是航向数据跳了，这时去转反而会把车转歪。
*/
void car_yaw_hold(float yaw0)
{
#if CAR_YAW_HOLD_ENABLE
	float error;

	if (!imu_angle_ready() || !dianji_move_flag) { return; }
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	delay_ms1(CAR_YAW_SETTLE_MS);            /* 等横移彻底停下并收到新航向 */
	imu_scan(fAcc, fGyro, fAngle);
	error = car_yaw_error(yaw0, fAngle[2]);
	if (error > -CAR_YAW_HOLD_TOL_DEG && error < CAR_YAW_HOLD_TOL_DEG) { return; }
	if (error > CAR_YAW_HOLD_MAX_DEG || error < -CAR_YAW_HOLD_MAX_DEG) { return; }

	car_turn_to_stable_time(yaw0, CAR_YAW_HOLD_TIME_MS);
#else
	(void)yaw0;
#endif
}

void PID_move1(int v_x,int v_y,int w,uint32_t time,int target,int pid_choose)
{   
	TIM_Cmd(TIM4, ENABLE); // 使能TIM4，定时器开始运行
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	flag = 0; //运动标志位归零
	if(pid_choose == 0){
		mypid.kp = 1.5;
		mypid.ki = 0.1;
		mypid.kd = 0.1;
	}else{
		mypid.kp = 1.5;
		mypid.ki = 0.001;
		mypid.kd = 0.125;
	}
	PID_DIL(v_x,v_y,w,time,target);
}

//定时读取陀螺仪角度数据进行PID计算
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{   
		if(real_time <= delay_time && dianji_move_flag){
			imu_scan(fAcc, fGyro, fAngle);
			feedbackValue = fAngle[2]; //反馈值
			pid_rate_deg_s = PID_GYRO_Z_SIGN * fGyro[2];
			pid_tick_count++;
			if(feedbackValue != feedbackValue_pre){ pid_yaw_value_changed_count++; }
			PID_Calc(&mypid, targetValue, feedbackValue); //进行PID计算，结果在output成员变量
			if(feedbackValue == feedbackValue_pre){
				same_time++;
			}
			feedbackValue_pre = fAngle[2];
			if(turn_stable_mode){
				if(mypid.error >= -TURN_STABLE_TOLERANCE_DEG &&
				   mypid.error <= TURN_STABLE_TOLERANCE_DEG &&
				   pid_rate_deg_s >= -TURN_STABLE_MAX_RATE_DEG_S &&
				   pid_rate_deg_s <= TURN_STABLE_MAX_RATE_DEG_S){
					car_move(0,0,0);
					turn_stable_count++;
					if(turn_stable_count >= TURN_STABLE_SAMPLE_COUNT){
						flag = 1;
					}else{
						flag = 0;
					}
				}else{
					turn_stable_count = 0U;
					if(mypid.output > -TURN_STABLE_MIN_SPEED &&
					   mypid.output < TURN_STABLE_MIN_SPEED){
						if(mypid.output > 0.0f){
							mypid.output = TURN_STABLE_MIN_SPEED;
						}else if(mypid.output < 0.0f){
							mypid.output = -TURN_STABLE_MIN_SPEED;
						}else{
							mypid.output = (mypid.error >= 0.0f) ?
							               TURN_STABLE_MIN_SPEED : -TURN_STABLE_MIN_SPEED;
						}
					}
					car_move(0,0,(int)mypid.output);
					flag = 0;
				}
			}else{
				car_move(vx,vy,mypid.output);
				if(turn_approach_mode){
					/* 第一段只负责把车转到目标附近：进了 ±CAR_TURN_REACHED_DEG 就交出控制权，
					   末段 PID_turn_stable 再用低速精修。这样把限速调慢也不会拖长整次转向，
					   也不会像原来那样明明已经到位还要空转到时间用完 */
					if(mypid.error >= -CAR_TURN_REACHED_DEG &&
					   mypid.error <= CAR_TURN_REACHED_DEG){
						if(turn_approach_count < CAR_TURN_REACHED_COUNT){
							turn_approach_count++;
						}
					}else{
						turn_approach_count = 0U;
					}
					flag = (turn_approach_count >= CAR_TURN_REACHED_COUNT) ? 1 : 0;
				}else{
					flag = 0;
				}
			}
		}else if(PID_control_flag == 0){
			car_move(0,0,0);
			flag =1;
		}
		real_time+=50; //每次加50ms
		circle++;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}



/*
	输入横纵移动的坐标，移动移动加速度和速度
	单位是mm
	一般设置加速度为70rpm，速度为50rpm
*/
void car_move_distance(float x,float y,int a_in,int speed_in)
{   
	float yaw0 = 0.0f;
	uint8_t yaw_hold = (ABS(y) >= CAR_YAW_HOLD_MIN_MM) ? 1U : 0U;

	delay_ms1(10);//防止堵塞can通信
	/* 一次定位是“纵向 x + 横移 y”两段：横移前先记下航向，两段走完一起拉回，
	   这样纵向段蹭偏的方向也一并纠回来。没有横移分量就完全按原来的流程走。 */
	if (yaw_hold) { yaw0 = car_yaw_mark(); }
	change_SNA1(a_in,speed_in);
	car_move_distance_y(x);
	car_move_delay(x,a_in,speed_in,1);
	
	delay_ms1(10);//防止堵塞can通信
	change_SNA1(a_in,speed_in);
	car_move_distance_x(y);
	car_move_delay(y,a_in,speed_in,0);
	if (yaw_hold) { car_yaw_hold(yaw0); }
}




//单位是mm
void PID_car_move_distance(float x,float y,float angel,float speed)
{	
	double fenliang = speed/pow((x*x+y*y),0.5);
	double time = 0;
	time = pow((x*x+y*y),0.5)*4;
	change_A(0);
	PID_move(y*fenliang,x*fenliang,0,time,angel,1);
}



void Time_Init()
{
    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 开启TIM4的时钟

    /*配置时钟源*/
    TIM_InternalClockConfig(TIM4);

    /*时基单元初始化*/
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // 定义结构体变量
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 计数器模式
    TIM_TimeBaseInitStructure.TIM_Period = 10000/2 - 1; // 计数周期，即ARR的值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 840 - 1; // 预分频器，即PSC的值
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; // 重复计数器
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure); // 配置TIM4的时基单元

    /*中断输出配置*/
    TIM_ClearFlag(TIM4, TIM_FLAG_Update); // 清除定时器更新标志位
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); // 开启TIM4的更新中断

    /*NVIC中断分组*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 配置NVIC为分组2

    /*NVIC配置*/
    NVIC_InitTypeDef NVIC_InitStructure; // 定义结构体变量
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; // 选择配置NVIC的TIM4线
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能NVIC线路
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 指定优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 指定响应优先级
    NVIC_Init(&NVIC_InitStructure); // 配置NVIC外设
}
