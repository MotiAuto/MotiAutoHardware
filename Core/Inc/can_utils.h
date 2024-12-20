/*
 * can_utils.h
 *
 *  Created on: Dec 10, 2024
 *      Author: motii
 */

#ifndef INC_CAN_UTILS_H_
#define INC_CAN_UTILS_H_

#include <stdint.h>
#include "main.h"

//extern uint8_t rxBuf[8];
//extern uint8_t dlc;
//extern uint8_t rxID;

void CAN_TX(uint32_t id, uint8_t *txBuf, CAN_HandleTypeDef* can)
{
	static CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	static uint32_t TxMailBox;

	if(HAL_CAN_GetTxMailboxesFreeLevel(can) > 0)
	{
		HAL_CAN_AddTxMessage(can, &TxHeader, txBuf, &TxMailBox);
	}
}

//void CAN_RX_Callback(CAN_HandleTypeDef *hcan)
//{
//	static CAN_RxHeaderTypeDef RxHeader;
//	uint8_t  rxData[8];
//
//	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rxData) == HAL_OK)
//	{
//		rxID = (RxHeader.IDE == CAN_ID_STD)? RxHeader.StdId : RxHeader.ExtId;
//		dlc = RxHeader.DLC;
//		rxBuf[0] = rxData[0];
//		rxBuf[1] = rxData[1];
//		rxBuf[2] = rxData[2];
//		rxBuf[3] = rxData[3];
//		rxBuf[4] = rxData[4];
//		rxBuf[5] = rxData[5];
//		rxBuf[6] = rxData[6];
//		rxBuf[7] = rxData[7];
//	}
//}


#endif /* INC_CAN_UTILS_H_ */
