/*
 * @Author: Yangyujian 2480383735@qq.com
 * @Date: 2023-11-22 18:37:35
 * @LastEditors: Yangyujian 2480383735@qq.com
 * @LastEditTime: 2023-11-27 10:07:55
 * @FilePath: \Motor_SV6001\Drivers\Motor\bsp_can.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __BSP_CAN__H
#define __BSP_CAN__H

#include "main.h"
#include "motor.h"
#include "can.h"

#define	LEFT_MOTOR_ID	    0x01
#define	RIGHT_MOTOR_ID	    0x02

#define MODE_LOCATION 0x01      //行规位置模式
#define MODE_SPEED 0x03         //速度模式
#define MODE_CURRENT 0x04       //电流模式


void CanFeedback_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle);

void BDW_openPDO(CAN_HandleTypeDef* hcan, uint16_t id);
void BDW_Enable(CAN_HandleTypeDef* hcan, uint16_t id, uint16_t mode);
void BDW_Disable(CAN_HandleTypeDef* hcan, uint16_t id);
void BDW_setSpeed(CAN_HandleTypeDef* hcan, uint16_t id, int16_t left_vel, int16_t right_vel);
void BDW_setCurrent(CAN_HandleTypeDef* hcan, uint16_t id, int16_t left_current, int16_t right_current);
#endif // !__BSP_CAN__H