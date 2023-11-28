#ifndef __MOTOR_H__
#define __MOTOR_H__

#define pi acos(-1)
#define INSIDE 	1
#define OUTSIDE 0
#define CARLENGTH 300 		//车体长度的一半 
#define CARWIDTH (279.0 / 2 + 97.0)		//车体宽度的一半 
#define L 64				//车身到轮子的距离 
#define L1 30				//连接轴到车轮的距离 
#define L2 33				//连接轴到舵机的距离 
#define MOTOR_L 197.5			//车中心到舵机的距离 
#define SHAFT 292.35	//连接轴的长度 
#define ANGLE_INIT 15		//舵机初始偏转角度 
#define TOWARDS INSIDE			//INSIDE:连接舵机内测	OUTSIDE:连接舵机外侧 

int float_to_uint(float, float, float, int);
float uint_to_float(int, float, float, int);

//void Exercise(float Angle, float V, float *left_servo_angle, float *left_vel, float *right_servo_angle, float *right_vel);
uint32_t micros(void);
void Car(uint16_t time);
void carDataInit(void);
#endif

