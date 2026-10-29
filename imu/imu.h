#ifndef __IMU_H
#define __IMU_H

void imu_init();
void imu_scan(float* fAcc,float* fGyro,float* fAngle);
/* 收到至少一帧包含航向角的数据后返回 1。 */
unsigned char imu_angle_ready(void);
void Uart2Send(unsigned char *p_data, unsigned int uiSize);
void Usart2Init(unsigned int uiBaud);

#endif
