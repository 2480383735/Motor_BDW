/*
 * @Author: Yangyujian 2480383735@qq.com
 * @Date: 2023-11-27 14:35:34
 * @LastEditTime: 2023-11-27 15:24:57
 * @LastEditors: Yangyujian 2480383735@qq.com
 * @Description: 
 * @FilePath: \BDW\Drivers\Motor\bsp_can.c
 * 未经允许,请勿转载!!!
 */
#include "bsp_can.h"
#include "main.h"
 

typedef struct bsp_can
{
    /* data */
    uint8_t		REPLE;			//反馈字节
	uint8_t 	MODE;			//工作模式
    uint8_t		ANGLE_L;		//角度低位
    uint8_t		ANGLE_H;		//角度高位
    uint8_t		VEL_L;			//速度低位
    uint8_t		VEL_H;			//速度高位
    uint8_t		CURRENT_L;		//电流低位
    uint8_t		CURRENT_H;		//电流高位
    uint8_t		STATUS;			//状态
}CAN_RxDataTypeDef_BDW;

CAN_RxDataTypeDef_BDW	rx_can_left_motor_data,rx_can_right_motor_data;				//舵机反馈数据
uint16_t				left_motor_feedback_angle, right_motor_feedback_angle;		//反馈角度
uint16_t				left_motor_feedback_vel, right_motor_feedback_vel;			//反馈速度
uint16_t				left_motor_send_angle, right_motor_send_angle;				//下发角度
int16_t					left_motor_difference_angle, right_motor_difference_angle;	//反馈角度与下发角度的差值
uint16_t				motor_allowable_error, motor_adjust_value;
float					motor_error_scale;


void CanFeedback_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;

  	sFilterConfig.FilterBank = 0;
 	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
 	sFilterConfig.FilterIdHigh = 0x0000;
 	sFilterConfig.FilterIdLow = 0x0000;
  	sFilterConfig.FilterMaskIdHigh = 0x0000;
  	sFilterConfig.FilterMaskIdLow = 0x0000;
  	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  	sFilterConfig.FilterActivation = ENABLE;
  	sFilterConfig.SlaveStartFilterBank = 14;
  	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

 /**
  * @name			HAL_CAN_RxFifo0MsgPendingCallback
  * @brief			Rx Fifo 0 message pending callback in non blocking mode
  * @param  		CanHandle: pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
  * @retval 		无
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef   	RxHeader;
  	uint8_t					RxData[8] = {0};
  /*获取接收CAN1数据*/
  if(hcan -> Instance == CAN1)
  {
  	HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &RxHeader, RxData);
	if(RxHeader.StdId	==  LEFT_MOTOR_ID && RxHeader.DLC	==	8)							//接收左舵机的反馈
	{
		rx_can_left_motor_data.REPLE		=	RxData[0];
		rx_can_left_motor_data.ANGLE_L		=	RxData[1];
		rx_can_left_motor_data.ANGLE_H		=	RxData[2];
		rx_can_left_motor_data.VEL_L		=	RxData[3];
		rx_can_left_motor_data.VEL_H		=	RxData[4];
		rx_can_left_motor_data.CURRENT_L	=	RxData[5];
		rx_can_left_motor_data.CURRENT_H	=	RxData[6];
		rx_can_left_motor_data.STATUS		=	RxData[7];
	}
	if(RxHeader.StdId	==	RIGHT_MOTOR_ID && RxHeader.DLC	==	8)							//接收来自右舵机的反馈
	{
		rx_can_right_motor_data.REPLE		=	RxData[0];
		rx_can_right_motor_data.ANGLE_L		=	RxData[1];
		rx_can_right_motor_data.ANGLE_H		=	RxData[2];
		rx_can_right_motor_data.VEL_L		=	RxData[3];
		rx_can_right_motor_data.VEL_H		=	RxData[4];
		rx_can_right_motor_data.CURRENT_L	=	RxData[5];
		rx_can_right_motor_data.CURRENT_H	=	RxData[6];
		rx_can_right_motor_data.STATUS		=	RxData[7];
	}
  }
}
/**************************PDO模式************************************/
/**
 * @description: 开启PDO模式
 * @param {CAN_HandleTypeDef*} hcan
 * @param {uint16_t} id
 * @return {*}
 */
void BDW_openPDO(CAN_HandleTypeDef* hcan, uint16_t id)
{
	uint8_t Data[2];
	Data[0] = 0x01;
	Data[1] = 0x00 + id;
	uint32_t  TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = 0x00;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 2;
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}
/**
 * @description: 电机使能
 * @param {CAN_HandleTypeDef*} hcan	
 * @param {uint16_t} id
 * @param {uint16_t} mode	电机控制模式;MODE_LOCATION:行规位置模式;MODE_SPEED:行规速度模式;MODE_CURRENT:电流(力矩)模式
 * @return {*}
 */
void BDW_Enable(CAN_HandleTypeDef* hcan, uint16_t id, uint16_t mode)
{
	uint8_t Data[5] = {0x0F, 0x00, 0x0F, 0x00, mode};
	uint32_t  TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = 0x200+id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 5;
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}
/**
 * @description: 电机失能
 * @param {CAN_HandleTypeDef*} hcan
 * @param {uint16_t} id
 * @return {*}
 */
void BDW_Disable(CAN_HandleTypeDef* hcan, uint16_t id)
{
	uint8_t Data[5] = {0x00, 0x00, 0x00, 0x00, 0xFF};
	uint32_t  TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = 0x200+id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 5;
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}
/**************************速度模式************************************/

/**
 * @description: 
 * @param {CAN_HandleTypeDef*} hcan
 * @param {uint16_t} id
 * @param {int16_t} left_vel 左电机速度
 * @param {int16_t} right_vel 右电机速度
 * @return {*}
 */
void BDW_setSpeed(CAN_HandleTypeDef* hcan, uint16_t id, int16_t left_vel, int16_t right_vel)
{
	uint8_t Data[8] = {0,0,0,0,0,0,0,0};
	uint8_t *left_vbuf, *right_vbuf;
	left_vbuf=(uint8_t*)&left_vel;
	right_vbuf=(uint8_t*)&right_vel;
	Data[0] = *left_vbuf;
	Data[1] = *(left_vbuf+1);
	Data[4] = *right_vbuf;
	Data[5] = *(right_vbuf+1);
	if(left_vel < 0)
	{
		Data[2] = 0xFF;
		Data[3] = 0xFF;
	}
	if(right_vel < 0)
	{
		Data[6] = 0xFF;
		Data[7] = 0xFF;	
	}
	uint32_t  TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = 0x300+id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}

/**************************电流(力矩)模式************************************/

/**
 * @description: 设置电机的电流
 * @param {CAN_HandleTypeDef*} hcan 
 * @param {uint16_t} id 
 * @param {int16_t} left_current 左电机电流
 * @param {int16_t} right_current 右电机电流值
 * @return {*}
 */
void BDW_setCurrent(CAN_HandleTypeDef* hcan, uint16_t id, int16_t left_current, int16_t right_current)
{
	uint8_t Data[8] = {0,0,0,0,0,0,0,0};
	uint8_t *left_abuf, *right_abuf;
	left_abuf=(uint8_t*)&left_current;
	right_abuf=(uint8_t*)&right_current;
	Data[0] = *left_abuf;
	Data[1] = *(left_abuf+1);
	Data[4] = *right_abuf;
	Data[5] = *(right_abuf+1);
	if(left_current < 0)
	{
		Data[2] = 0xFF;
		Data[3] = 0xFF;
	}
	if(right_current < 0)
	{
		Data[6] = 0xFF;
		Data[7] = 0xFF;	
	}
	uint32_t  TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = 0x300+id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}