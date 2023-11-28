/*
 * @Author: Yangyujian 2480383735@qq.com
 * @Date: 2023-11-27 14:35:34
 * @LastEditTime: 2023-11-27 15:13:28
 * @LastEditors: Yangyujian 2480383735@qq.com
 * @Description: 
 * @FilePath: \BDW\Drivers\Motor\motor.c
 * 未经允许,请勿转载!!!
 */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "motor.h"
#include "bsp_uart.h"
#include  "can.h"

int i = 0;


typedef struct motor
{
	float car_V;		//车辆速度
	float car_A;		//车辆偏转角度
	float left_wheel_V;		//左车轮速度
	float left_wheel_A;		//左车轮偏转角度
	float right_wheel_V;		//右车轮速度
	float right_wheel_A;		//右车轮偏转角度
	float left_servo_V;		//左舵机转向速度
	float left_servo_A;		//左舵机偏转角度
	float right_servo_V;		//右舵机转向速度
	float right_servo_A;		//右舵机偏转角度
	float left_R;
	float car_R;
	float right_R;
}Motor;

Motor data[20];			//定义结构体数组存储数据

__STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
static uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  GXT_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (GXT_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}
//获取系统时间，单位us
uint32_t micros(void)
{
  return getCurrentMicros();
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief 
 * 
 * @param Angle 车辆偏转角度
 * @param V 车辆速度
 * @param left_servo_angle 左侧(内测)舵机偏转角度
 * @param left_vel 左侧(内测)车轮速度
 * @param right_servo_angle 右侧(外侧)舵机偏转角度 
 * @param right_vel 右侧(外侧)车轮速度
 */
void new_cal_turn_left_right_deal(float Angle, float V, float *left_servo_angle, float *left_vel, float *right_servo_angle, float *right_vel)
{
	float R,R1,R2,angle,inside_wheel_angle,outside_wheel_angle,outside_servo_angle,inside_servo_angle;
	float tab,p,q,t;
	float X1,Y1;
	float a,d,h,y0,yc;
    float outside_vel;
    float inside_vel;
    char direction=0;
	if(Angle == 0)
	{
		*left_servo_angle = 0;
		*right_servo_angle = 0;
	}

	data[i].car_V =  V;
    if(Angle<0)
    {
        Angle = -Angle;
        direction = 1;
    }
	angle = Angle / 180 * pi;	//整车角度 
	R = CARLENGTH /  cos(pi / 2 - angle); //整车半径 
	data[i].car_R = R;
    R1 = sqrt(R*R + CARWIDTH*CARWIDTH - 2*R*CARWIDTH*cos(pi - angle)) + L;//外轮半径 
	outside_wheel_angle = acos(((R1-L)*(R1-L) + CARWIDTH*CARWIDTH - R*R) / (2*(R1-L)*CARWIDTH));//外轮角度 
	outside_vel = R1 / R * V;
    data[i].right_R =  R1;


    R2 = sqrt(R*R + CARWIDTH*CARWIDTH -2*R*CARWIDTH*cos(angle)) - L;		//内轮半径 
	inside_wheel_angle = pi - acos(((R2+L)*(R2+L) + CARWIDTH*CARWIDTH - R*R) / (2*(R2+L)*CARWIDTH));
	tab  = acos(((R2+L)*(R2+L) + R*R - CARWIDTH*CARWIDTH) / (2*(R2+L)*R));
    p = R * ((R2) / (R2 + L)) - R2 / cos(tab);
    q = CARWIDTH * ((R2) / (R2 + L));
    t = sqrt(p*p + q*q -2*p*q*cos(angle));
    inside_wheel_angle = pi / 2 - acos((t*t + q*q - p*p) / (2*t*q));//内轮角度

	inside_vel = R2 / R * V;

	data[i].left_R = R2;
	//左舵机角度计算
	X1 = CARWIDTH - L1*cos(inside_wheel_angle);				//车轮连接点的横坐标 
    Y1 = CARLENGTH + L1*sin(inside_wheel_angle);				//车轮连接点的纵坐标 
    d = sqrt((MOTOR_L-X1)*(MOTOR_L-X1) + Y1*Y1);
    a = (SHAFT*SHAFT - L2*L2  + d*d) / (2*d);
    h = sqrt((SHAFT*SHAFT) - (a*a));
    y0 = Y1 +(a/d)*(-1)*Y1;
    yc = y0 + (h/d)*(MOTOR_L-X1);
	inside_servo_angle = asin(yc/L2) * 180 / pi - ANGLE_INIT;	//内圈舵机偏转角度 

	//	(X2,Y2) = (MOTOR_L, 0)
	//右舵机角度计算
	X1 = CARWIDTH - L1*cos(outside_wheel_angle);
    Y1 = CARLENGTH - L1*sin(outside_wheel_angle);
    d = sqrt((MOTOR_L-X1)*(MOTOR_L-X1) + Y1*Y1);
    a = (SHAFT*SHAFT - L2*L2  + d*d) / (2*d);
    h = sqrt((SHAFT*SHAFT) - (a*a));
    y0 = Y1 +(a/d)*(-1)*Y1;
    yc = y0 + (h/d)*(MOTOR_L-X1);
    outside_servo_angle = (asin(yc/L2) * 180 / pi) * (-1) + ANGLE_INIT;		//外圈偏转角度 

    if(direction==1)//turn right
    {        
        *right_vel = inside_vel;
        *left_vel = outside_vel;

        *left_servo_angle = -outside_servo_angle;
        *right_servo_angle = -inside_servo_angle;
    }
    else if(direction==0)
    {
        *right_vel= outside_vel;
        *left_vel=inside_vel;

        *left_servo_angle = inside_servo_angle;
        *right_servo_angle = outside_servo_angle;
    }
}

//处理遥控器发送的值
/**************************
 * angle_1:当前车辆偏转角度
 * angle_2:车辆目标偏转角度
****************************/
/**
 * @description: 
 * @param {float} *angle_1
 * @param {float} angle_2
 * @param {float} *vel_1
 * @param {float} vel_2
 * @return {*}
 */
void  carAngleChanges(float *angle_1, float angle_2, float *vel_1, float vel_2)
{
	//获取当前车辆的实际角度
//	100ms/2  150ms/3
	//获取当前角度和目标角度的差值
	float angle = angle_2 - *angle_1;
	float vel = vel_2 - *vel_1;
	//改变车辆当前的角度,每次最大变化3
	if (angle > 2)
		*angle_1 += 2;
	else if (angle < -2)
		*angle_1 -= 2;
	else
		*angle_1 = angle_2;
	//改变车辆当前的速度,每次最大变化5
	if (vel > 5)
		*vel_1 += 5;
	else if (vel < -5)
		*vel_1 -= 5;
	else
		*vel_1 = vel_2;
}
/*
	第一步:获取车辆变化的角度和速度并拆分
	第二步:获取舵机当前的位置
	第三步:计算舵机的偏航角度
	第四步:计算车轮的速度
		方式1:根据目标值设置车轮的速度(超前)
		方式2:根据当前值设置车轮的速度(滞后)
		方式3:根据实时反馈设置车轮速度(进程缓慢)
	第五步:计算舵机的速度
*/
	//左侧舵机需要偏转的角度 left_servo_difference = *left_servo_angle - left_pos
	//右侧舵机需要偏转的角度 right_servo_difference = *right_servo_angle - right_pos
	//下一轮执行的时间time(设为理想时间)
	//左侧舵机的速度left_servo_vel = left_servo_difference / time
	//右侧舵机的速度right_servo_vel = right_servo_difference / time

	//将左侧舵机和右侧舵机的目标位置与速度发送给舵机
float angle , Vel, left_servo_angle, left_vel, right_servo_angle, right_vel;
extern rc_info_t rc;
float left_pos, right_pos ,left_servo_vel, right_servo_vel;
float angle_now,vel_now;
float left_x,right_x,left_servo_vel_now,right_servo_vel_now; 

void carDataInit(void)
{
	angle_now = 0;
	vel_now = 0;
	left_servo_angle = 0;
	right_servo_angle =0;
}

void Car(uint16_t time)
{
	/*Vel = ((float)(rc.ch3)) / 17.4;		//遥控器发送的目标速度,范围-45 - 45
	if(((float)(rc.ch3)) <= -10 && ((float)(rc.ch3)) >= -30)
		Vel = 0;
	angle = ((float)(rc.ch4)) / 250;	//遥控器发送的目标角度,范围-3.14 - 3.14  */

	//float at = 50.0;	//设加速过程时间为50ms
	angle = 33; //给定偏转角度
	Vel = 114;	//给定车辆速度

	left_pos = left_servo_angle;		//左侧舵机当前的位置
	right_pos = right_servo_angle;		//右侧舵机当前的位置

	left_servo_vel_now = left_servo_vel;	//左舵机当前的速度
	right_servo_vel_now = right_servo_vel;	//右舵机当前的速度

	//改变车辆的偏转角度和前进速度
	carAngleChanges(&angle_now, angle, &vel_now, Vel);
	//计算舵机的偏转角度和车轮的前进速度
	new_cal_turn_left_right_deal(angle_now, 10, &left_servo_angle, &left_vel, &right_servo_angle, &right_vel);
	//计算左右舵机需要偏转角度的差值,选取合适的速度匹配左右舵机的时间差值

	left_x = left_servo_angle - left_pos; //左舵机转向的差值
	right_x = right_servo_angle - right_pos; //右舵机转向的差值

	left_servo_vel = (left_x / time) / 360* 600 * 100 * 1000;
	right_servo_vel = (right_x / time) / 360 * 600 * 100 * 1000;;

	data[i].car_A = angle_now;
	data[i].left_wheel_V = left_vel;
	data[i].right_wheel_V = right_vel;
	data[i].left_servo_V = left_servo_vel;
	data[i].left_servo_A = left_servo_angle;
	data[i].right_servo_V = right_servo_vel;
	data[i].right_servo_A = right_servo_angle;
	i++;
	//发送位置和速度给舵机

	//发送速度给电机

}

