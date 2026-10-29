#include "stm32f4xx.h"                  // Device header
#include "Emm_V5.h"
#include "../SYSTEM/delay/delay.h"  // 使用工程实际编译的延时模块，也声明 delay_ms1
#include "OLED.h"
#include "math.h"
#include "yyb_move.h"

/*
	ZDT X42S 驱动器 S_Vel_IS（手册 P19）：Enable 时通讯指令里的速度值会缩小 10 倍，
	实际转速 = 指令值 / 10。实车当前设置为 Disable，因此指令值就是实际 RPM，
	下面的速度倍率使用 1.0。
	VEL_SCALE_POS 用于位置模式（FD：car_move1/2、升降、平推）的等待时间；
	VEL_SCALE_VEL 用于速度模式（F6：航向 PID 的 car_move）的转速换算。
*/
#define VEL_SCALE_POS   1.0f
#define VEL_SCALE_VEL   1.0f
#define PULSE_PER_REV   3200.0f   /* 16 细分，手册 P57：3200 个脉冲电机转一圈 */
#define CHASSIS_X_PULSE_PER_MM 14.7f    /* 当前实车复调值 */
#define CHASSIS_Y_PULSE_PER_MM 13.6073f /* 1440 mm 命令实测 1090 mm：10.3 * 1440 / 1090 */


int  car_move_speed = 50;
int  a1 = 70;
int  a  = 230;
volatile uint32_t can_position_batch_abort_count = 0U;
static float pingtui_distance = 0.0f;
static float current_pos = 0.0f; 
static float current_pos1 = 0.0f;  //升降的位置
double car_move_delay(float distance,uint16_t a_of_car,uint16_t speed_of_car,uint8_t direct);
double shengjiang_move_delay(float quanshu,uint16_t a,uint16_t speed);
void change_A(int a_in);
void change_SNA1(int a,int speed);
/* 横移前后的航向稳定，实现在 ../Hardware/pid.c */
float car_yaw_mark(void);
void car_yaw_hold(float yaw0);

/* 横移距离小于它就不做航向稳定：本来就没动，没必要再转一下 */
#define LATERAL_YAW_HOLD_MIN_MM  1.0f

void car_move(int v_x,int v_y,int w)
{
	int speed[5];
	int dt=1;
	if (!CHASSIS_MOTOR_OUTPUT_ENABLE) {
		(void)v_x;
		(void)v_y;
		(void)w;
		return;
	}
	/*
	 * 2026-10-21 实车轮向：四轮相同逻辑正转均对应驱动器 dir=1。
	 * 因此正速度统一发 dir=1，负速度统一发 dir=0；否则纯 w
	 * 指令会让四轮同向前转，无法形成左右轮反向的原地转向。
	 */
	
	speed[1] = +v_x + v_y-w;
	speed[2] = -v_x + v_y+w;
	speed[3] = -v_x + v_y-w;
	speed[4] = +v_x + v_y+w;
	if(speed[1]>=0){
		Emm_V5_Vel_Control(2, 1, speed[1], a, false);//正向
	}
	else{
	    Emm_V5_Vel_Control(2, 0, -speed[1], a, false);   
	}
	delay_ms(dt);
	if(speed[2]>=0){
		Emm_V5_Vel_Control(1, 1, speed[2], a, false);//正向	
	}
	else{
	    Emm_V5_Vel_Control(1, 0, -speed[2], a, false);  
	}
	delay_ms(dt);
	if(speed[3]>=0){
		Emm_V5_Vel_Control(3, 1, speed[3], a, false);//正向	
	}
	else{
	    Emm_V5_Vel_Control(3, 0, -speed[3], a, false);  
	}
	delay_ms(dt);
	if(speed[4]>=0){
		Emm_V5_Vel_Control(4, 1, speed[4], a, false);//正向	
	}
	else{
	    Emm_V5_Vel_Control(4, 0, -speed[4], a, false); 
	}
	delay_ms(dt);
}

//输入为x轴需要移动的距离,单位是mm
void car_move_distance_x(float x)
{
	int speed[5];
	uint8_t batch_ok = 1U;
	int dt=5;
	int v_x;
	int quanshu = ABS(x) * CHASSIS_X_PULSE_PER_MM;
	if (!CHASSIS_MOTOR_OUTPUT_ENABLE) {
		(void)x;
		return;
	}
    v_x = car_move_speed*x/ABS(x);
	speed[1] = +v_x ;
	speed[2] = -v_x ;
	speed[3] = -v_x ;
	speed[4] = +v_x ;
	if(speed[1]>=0){
		Emm_V5_Pos_Control(2, 1, speed[1], a1, quanshu,false,true);//正向
		batch_ok &= can_tx_last_cmd_ok;
	}
	else{
	    Emm_V5_Pos_Control(2, 0, -speed[1], a1, quanshu,false,true);   
		batch_ok &= can_tx_last_cmd_ok;
	}
	delay_ms(dt);
	if(speed[2]>=0){
		Emm_V5_Pos_Control(1, 1, speed[2], a1,quanshu,false, true);//正向	
		batch_ok &= can_tx_last_cmd_ok;
	}
	else{
	    Emm_V5_Pos_Control(1, 0, -speed[2], a1,quanshu,false,true);  
		batch_ok &= can_tx_last_cmd_ok;
	}
	delay_ms(dt);
	if(speed[3]>=0){
		Emm_V5_Pos_Control(3, 1, speed[3], a1, quanshu,false,true);//正向	
		batch_ok &= can_tx_last_cmd_ok;
	}
	else{
	    Emm_V5_Pos_Control(3, 0, -speed[3], a1, quanshu,false,true);  
		batch_ok &= can_tx_last_cmd_ok;
	}
	delay_ms(dt);
	if(speed[4]>=0){
		Emm_V5_Pos_Control(4, 1, speed[4], a1, quanshu,false,true);//正向	
		batch_ok &= can_tx_last_cmd_ok;
	}
	else{
	    Emm_V5_Pos_Control(4, 0, -speed[4], a1, quanshu,false,true); 
		batch_ok &= can_tx_last_cmd_ok;
	}
	delay_ms(dt);
	/* 位置命令以 snF=true 缓存，必须发同步广播才会执行；即使个别帧失败也要发，
	   否则过期目标会留在驱动器里，被下一次同步放出来，表现为某一台多转一段 */
	Emm_V5_Synchronous_motion(0x00);
	if (!batch_ok) {
		++can_position_batch_abort_count;
	}
}

//输入为y轴需要移动的距离,单位是mm
void car_move_distance_y(float y)
{
	int speed[5];
	uint8_t batch_ok = 1U;
	int dt=5;
	int v_y;
	int quanshu = ABS(y) * CHASSIS_Y_PULSE_PER_MM;
	if (!CHASSIS_MOTOR_OUTPUT_ENABLE) {
		(void)y;
		return;
	}
    v_y = car_move_speed*y/ABS(y);
	speed[1] = v_y;
	speed[2] = v_y;
	speed[3] = v_y;
	speed[4] = v_y;
	if(speed[1]>=0){
		Emm_V5_Pos_Control(2, 1, speed[1], a1, quanshu,false,true);//正向
		batch_ok &= can_tx_last_cmd_ok;
	}
	else{
	    Emm_V5_Pos_Control(2, 0, -speed[1], a1, quanshu,false,true);   
		batch_ok &= can_tx_last_cmd_ok;
	}
	delay_ms(dt);
	if(speed[2]>=0){
		Emm_V5_Pos_Control(1, 1, speed[2], a1,quanshu,false, true);//正向	
		batch_ok &= can_tx_last_cmd_ok;
	}
	else{
	    Emm_V5_Pos_Control(1, 0, -speed[2], a1,quanshu,false, true);  
		batch_ok &= can_tx_last_cmd_ok;
	}
	delay_ms(dt);
	if(speed[3]>=0){
		Emm_V5_Pos_Control(3, 1, speed[3], a1, quanshu,false,true);//正向	
		batch_ok &= can_tx_last_cmd_ok;
	}
	else{
	    Emm_V5_Pos_Control(3, 0, -speed[3], a1, quanshu,false, true);  
		batch_ok &= can_tx_last_cmd_ok;
	}
	delay_ms(dt);
	if(speed[4]>=0){
		Emm_V5_Pos_Control(4, 1, speed[4], a1, quanshu,false,true);//正向	
		batch_ok &= can_tx_last_cmd_ok;
	}
	else{
	    Emm_V5_Pos_Control(4, 0, -speed[4], a1, quanshu,false,true); 
		batch_ok &= can_tx_last_cmd_ok;
	}
	delay_ms(dt);
	/* 位置命令以 snF=true 缓存，必须发同步广播才会执行；即使个别帧失败也要发，
	   否则过期目标会留在驱动器里，被下一次同步放出来，表现为某一台多转一段 */
	Emm_V5_Synchronous_motion(0x00);
	if (!batch_ok) {
		++can_position_batch_abort_count;
	}
}

//当y>0时左移
void car_move1(float x,float y,int a_in,int car_speed)
{
	delay_ms1(10);
	change_SNA1(a_in,car_speed);
	car_move_distance_y(-y);
	car_move_delay(y,a_in,car_speed,1);
}

//当x>0时左移
void car_move2(float x,float y,int a_in,int car_speed)
{
	float yaw0 = 0.0f;
	uint8_t yaw_hold = (ABS(x) >= LATERAL_YAW_HOLD_MIN_MM) ? 1U : 0U;

	delay_ms1(10);
	change_SNA1(a_in,car_speed);
	/* 横移走位置模式，过程中没有航向闭环：先记下当前航向，走完再拉回来，
	   和每次转完都做一次 PID_turn_stable 是同一套稳定方式。 */
	if (yaw_hold) { yaw0 = car_yaw_mark(); }
	car_move_distance_x(-x);
	car_move_delay(x,a_in,car_speed,0);
	if (yaw_hold) { car_yaw_hold(yaw0); }
}

//当y>0时左移//仅执行任务，不计时
void car_move3(float x,float y,int a_in,int car_speed)
{
	delay_ms1(10);
	change_SNA1(a_in,car_speed);
	car_move_distance_y(-y);
}

//当x>0时左移//不计时返回，但会等这一程走完再稳定航向
void car_move4(float x,float y,int a_in,int car_speed)
{
	float yaw0 = 0.0f;
	uint8_t yaw_hold = (ABS(x) >= LATERAL_YAW_HOLD_MIN_MM) ? 1U : 0U;

	delay_ms1(10);
	change_SNA1(a_in,car_speed);
	if (yaw_hold) { yaw0 = car_yaw_mark(); }
	car_move_distance_x(-x);
	/* 位置指令是异步的，不先等它走完就去测航向会在运动中途插进速度指令，
	   所以这里补上 car_move2 里那句 car_move_delay，再拉回航向。 */
	if (yaw_hold) {
		car_move_delay(x,a_in,car_speed,0);
		car_yaw_hold(yaw0);
	}
}


double shengjiang_control(int target_pos, uint16_t speed, uint8_t accel)
{   
	if(target_pos >=135.0f){
		target_pos =135.0f;
	}
	if (target_pos <0.0f){
        OLED_ShowString(1, 1, "error:OUT");
        while(1){}
    }
	delay_ms1(10);//放置堵塞can通信
	   // 计算需要移动的距离和方向
	int move_distance = target_pos - current_pos1;
    uint8_t dir = (move_distance > 0) ? 0 : 1;
    
    // 计算需要的脉冲数（取绝对值）
    uint32_t pulses = (uint32_t)(ABS(move_distance) * 80);

    // 执行运动控制
	if(target_pos != current_pos1){
		Emm_V5_Pos_Control(5, dir, speed, accel, pulses, false, false);
	}
    shengjiang_move_delay(2*pulses, accel, speed);

    // 更新当前位置
    current_pos1 = target_pos;
	return shengjiang_move_delay(1.0f*pulses, accel, speed);
}


/*
	dir = 0时，向前运动,距离不能超过60mm
	dir = 1时，向后运动,距离不能超过120mm
	25.465脉冲大概对应1mm
	以最高处为基准
	滑道最长
*/
double pingtui_control(float target_pos, uint16_t speed, uint8_t accel)
{   
	if (target_pos > 65.0f || target_pos < -122.0f){
        OLED_ShowString(1, 1, "error:OUT");
        while(1){}
    }
	delay_ms1(10);//放置堵塞can通信
	   // 计算需要移动的距离和方向
    float move_distance = target_pos - current_pos;
    uint8_t dir = (move_distance > 0) ? 0 : 1; // 0=向前(上升), 1=向后(下降)
    
    // 计算需要的脉冲数（取绝对值）
    uint32_t pulses = (uint32_t)(fabs(move_distance) * 25.465f);

    // 执行运动控制
	if(target_pos != current_pos){
		Emm_V5_Pos_Control(6, dir, speed, accel, pulses, false, false);
	}
    shengjiang_move_delay(2*pulses, accel, speed);

    // 更新当前位置
    current_pos = target_pos;
	return shengjiang_move_delay(1.0f*pulses, accel, speed);
}

void pingtui_reset(void)
{   
	pingtui_control(0.0f,1000,200);
}

/*
	//0代表x轴
	//1代表y轴
	根据输入的速度和加速度和路程
	delay一定的时间，用来空过两个连续的位置坐标指令
	放置前后指令的覆盖
*/
/* 手册 P53/P57：加速度 acc != 0 时，每 (256-acc)*50us 增减 1RPM；acc = 0 为直接启动 */
static double accel_rpm_per_s(uint16_t acc)
{
	if (acc == 0U) { return 0.0; }
	if (acc > 255U) { acc = 255U; }
	return 20000.0 / (double)(256U - acc);
}

/* 位置模式走完 pulses 个脉冲所需时间（ms），加速段与减速段加速度相同 */
static double pos_move_time_ms(double pulses, double vel_rpm, uint16_t acc)
{
	double rev, v, a, ramp_rev;

	if (pulses <= 0.0) { return 0.0; }
	v = vel_rpm / 60.0;                        /* 转/秒 */
	if (v <= 0.0) { return 0.0; }
	a = accel_rpm_per_s(acc) / 60.0;           /* 转/秒^2 */
	rev = pulses / PULSE_PER_REV;
	if (a <= 0.0) { return 1000.0 * rev / v; } /* 直接启动，匀速走完 */
	ramp_rev = (v * v) / (2.0 * a);            /* 单侧加减速段走过的圈数 */
	if (rev <= 2.0 * ramp_rev) {               /* 还没加速到目标速度就要减速 */
		return 1000.0 * 2.0 * sqrt(rev / a);
	}
	return 1000.0 * (2.0 * (v / a) + (rev - 2.0 * ramp_rev) / v);
}


double car_move_delay(float distance,uint16_t a_of_car,uint16_t speed_of_car,uint8_t direct)
{
	double quanshu = 0;
	double time = 0;

	if(direct == 0){
		quanshu = ABS(distance) * CHASSIS_X_PULSE_PER_MM;
	}else{
		quanshu = ABS(distance) * CHASSIS_Y_PULSE_PER_MM;
	}
	time = pos_move_time_ms(quanshu, (double)speed_of_car * VEL_SCALE_POS, a_of_car);
	if(ABS(distance) <= 2){time = 200;}
	if(time < 50){time = 50;}
	delay_ms1((int)time);
	return time;
}


/*
	根据输入的速度和加速度和路程
	delay一定的时间，用来空过两个连续的位置坐标指令
	放置前后指令的覆盖
*/
double shengjiang_move_delay(float quanshu,uint16_t a,uint16_t speed)
{
	double time = pos_move_time_ms(ABS(quanshu), (double)speed * VEL_SCALE_POS, a);

	if(time < 200){time = 200;}//最少也要等待200ms
	return time;
}


/***************************
	改变PID_move的速度大小
	输入为加速度的大小
****************************/
void change_A(int a_in)
{
	a = a_in;
}


/******************************
	改变car_move的加速度和速度大小
	前面输入加速度，后面输入速度
*******************************/
void change_SNA1(int a,int speed)
{
	a1 = a;
	car_move_speed = speed;
}
