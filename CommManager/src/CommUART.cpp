/*
 * CommSPI.cpp
 *
 *  Created on: 30 gru 2022
 *      Author: Paulina
 */

#include "CommUART.hpp"

CommUART::CommUART(UART_HandleTypeDef *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx, CommModeTypeDef CommMode)
:CommBaseClass(hint, hdmaRx, CommMode), _hdmaTx{hdmaTx}
{}

HAL_StatusTypeDef CommUART::__CheckIfInterfaceFree(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo)
{
	HAL_StatusTypeDef ret = HAL_BUSY;
	//Disallow async for now
	if(_Handle->gState != HAL_UART_STATE_READY)
	{
		return ret;
	}
	return HAL_OK;
}

HAL_StatusTypeDef CommUART::__CheckIfFreeAndSendRecv(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo)
{
	HAL_StatusTypeDef ret = HAL_BUSY;
	if(_commType != COMM_DUMMY)
	{
		__CheckAndSetCSPinsGeneric(MsgInfo);
	}
	switch(MsgInfo->eCommType)
	{
		case COMM_INT_RX:
		{
			if(_commType == COMM_DMA)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_UARTEx_ReceiveToIdle_DMA(_Handle, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
//				ret = HAL_UARTEx_ReceiveToIdle_IT(_Handle, MsgInfo->pRxData, MsgInfo->len);
				ret = HAL_UART_Receive_IT(_Handle, MsgInfo->pRxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_UART_Receive(_Handle, MsgInfo->pRxData, MsgInfo->len, 1000);
			}
			else
			{
				ret = HAL_ERROR;
			}
		}
		break;
		case COMM_INT_TX:
		{
			if(_commType == COMM_DMA)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_UART_Transmit_DMA(_Handle, MsgInfo->pTxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_UART_Transmit_IT(_Handle, MsgInfo->pTxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_UART_Transmit(_Handle, MsgInfo->pTxData, MsgInfo->len, 1000);
			}
			else
			{
				ret = HAL_ERROR;
			}
		}
		break;
		default:
			ret = HAL_ERROR;
			break;
	}
	return ret;
}

HAL_StatusTypeDef CommUART::PushMessageIntoQueue(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo)
{

	if(MsgInfo->TransactionStatus !=0)
	{
		*MsgInfo->TransactionStatus = HAL_BUSY;
	}
	if(MsgInfo->IntHandle->Instance != this->_Handle->Instance)
	{
		return HAL_ERROR;
	}
	if(uxQueueMessagesWaitingFromISR(_MsgQueue) >= MAX_MESSAGE_NO_IN_QUEUE)
	{
		return HAL_ERROR;
	}

	if(MsgInfo->eCommType == COMM_INT_RX){
		return this->__CheckForNextRxCommRequestAndStart(MsgInfo);
	}
	else{
		return this->__CheckForNextCommRequestAndStart(MsgInfo);
	}

}

HAL_StatusTypeDef CommUART::__CheckForNextRxCommRequestAndStart(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo)
{
	HAL_StatusTypeDef ret = HAL_OK;
	MessageInfoTypeDef<UART_HandleTypeDef> Msg = {0};
	MessageInfoTypeDef<UART_HandleTypeDef> *pMsg = &Msg;
	if(uxQueueMessagesWaitingFromISR(_RxMsgQueue) == 0 && MsgInfo != NULL)
	{
		if(this->__CheckIfInterfaceFree(MsgInfo) == HAL_OK){
			//send message
			if(xQueueSendToBackFromISR(_RxMsgQueue, MsgInfo, NULL) != pdPASS)
			{
				return HAL_ERROR;
			}
			ret = this->__CheckIfFreeAndSendRecv(MsgInfo);
		}
	}
	else
	{
		if(MsgInfo != NULL)
		{
			if(xQueueSendToBackFromISR(_RxMsgQueue, pMsg, NULL) != pdPASS)
			{
				return HAL_ERROR;
			}
			pMsg = MsgInfo;
		}
		if(xQueuePeekFromISR(_RxMsgQueue, pMsg) == pdPASS){
			ret = this->__CheckIfFreeAndSendRecv(pMsg);
		}
	}
	return ret;

}

HAL_StatusTypeDef CommUART::MsgReceivedRxCB(UART_HandleTypeDef *hint)
{
	MessageInfoTypeDef<UART_HandleTypeDef> Msg = {0};
	if(xQueuePeekFromISR(_RxMsgQueue, &Msg) != pdTRUE)
	{
		Error_Handler();
	}
	return this->MsgReceivedCB(hint, Msg.len);
}

HAL_StatusTypeDef CommUART::MsgReceivedCB(UART_HandleTypeDef *hint, uint16_t len)
{
	MessageInfoTypeDef<UART_HandleTypeDef> Msg = {0};
	//remove item from queue
	if(xQueueReceiveFromISR(_RxMsgQueue, &Msg, NULL) != pdTRUE)
		{
			Error_Handler();
		}
	//Clear CS Pin if any
	if(Msg.GPIOx != 0){
		HAL_GPIO_WritePin(Msg.GPIOx, Msg.GPIO_PIN, CSn_INACTIVE_PIN_STATE);
	}
	//Set transaction status
	if(Msg.TransactionStatus != 0){
		*Msg.TransactionStatus = HAL_OK;
	}
	//set len
	Msg.len = len;
	switch(Msg.eCommType)
	{
		case COMM_INT_RX:
		{
			if(Msg.pRxCompletedCB != 0)
				{
					Msg.pRxCompletedCB(&Msg);
				}
		}
			break;
		case COMM_INT_TX:
		{
			if(Msg.pTxCompletedCB != 0)
				{
					Msg.pTxCompletedCB(&Msg);
				}
		}
			break;
		case COMM_INT_TXRX:
		{
			if(Msg.pRxCompletedCB != 0)
				{
					Msg.pRxCompletedCB(&Msg);
				}
			if(Msg.pTxCompletedCB != 0)
				{
					Msg.pTxCompletedCB(&Msg);
				}
		}
			break;
		default:
			break;
	}
	if(Msg.pCB != 0)
	{
		Msg.pCB(&Msg);
	}

	if(Msg.eCommType == COMM_INT_RX){
		return this->__CheckForNextRxCommRequestAndStart(NULL);
	}
	else{
		return this->__CheckForNextCommRequestAndStart(NULL);
	}
}



