/*
 * CommBaseClass.cpp
 *
 *  Created on: 30 gru 2022
 *      Author: Paulina
 */
#include "CommBaseClass.hpp"

template<typename T>
CommBaseClass<T>::CommBaseClass(T *hint, DMA_HandleTypeDef *hdmaRx, CommModeTypeDef CommMode)
:_Handle{hint}, hdmaRx{hdmaRx}, _commType{CommMode}, GPIO_PIN{0}, GPIOx{NULL}
{
	_MsgQueue = xQueueCreateStatic(MAX_MESSAGE_NO_IN_QUEUE,
										sizeof(MessageInfoTypeDef<T>),
										_xQueueStaticBuffer,
										&_xStQueue );
}


template<typename T>
HAL_StatusTypeDef CommBaseClass<T>::__CheckAndSetCSPinsGeneric(MessageInfoTypeDef<T> *MsgInfo)
{

		//Check if pin is already set
		if((GPIO_PIN == MsgInfo->GPIO_PIN) && (GPIOx == MsgInfo->GPIOx))
		{
			//if so return
			return HAL_OK;
		}
		else
		{
			//if not reset pin states and set new ones
			if(GPIOx != NULL)
			{
				HAL_GPIO_WritePin(GPIOx, GPIO_PIN, CSn_INACTIVE_PIN_STATE);
			}
			if(MsgInfo->GPIOx == 0)
			{
				return HAL_OK;
			}
			HAL_GPIO_WritePin(MsgInfo->GPIOx, MsgInfo->GPIO_PIN, CSn_ACTIVE_PIN_STATE);
			return HAL_OK;
		}

}

template<class T>
HAL_StatusTypeDef CommBaseClass<T>::__CheckForNextCommRequestAndStart(MessageInfoTypeDef<T> *MsgInfo)
{
	HAL_StatusTypeDef ret = HAL_OK;
	MessageInfoTypeDef<T> Msg = {0};
	MessageInfoTypeDef<T> *pMsg = &Msg;
	if(uxQueueMessagesWaitingFromISR(_MsgQueue) == 0 && MsgInfo != NULL)
	{
		if(this->__CheckIfInterfaceFree(MsgInfo) == HAL_OK){
			//send message
			if(xQueueSendToBackFromISR(_MsgQueue, MsgInfo, NULL) != pdPASS)
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
			if(xQueueSendToBackFromISR(_MsgQueue, pMsg, NULL) != pdPASS)
			{
				return HAL_ERROR;
			}
			pMsg = MsgInfo;
		}
		if(xQueuePeekFromISR(_MsgQueue, pMsg) == pdPASS){
			if(pMsg->IntHandle != NULL){
				ret = this->__CheckIfFreeAndSendRecv(pMsg);
			}
		}
	}
	return ret;
}

template<typename T>
HAL_StatusTypeDef CommBaseClass<T>::PushMessageIntoQueue(MessageInfoTypeDef<T> *MsgInfo)
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

	return this->__CheckForNextCommRequestAndStart(MsgInfo);
}

template<typename T>
HAL_StatusTypeDef CommBaseClass<T>::MsgReceivedCB(T *hint)
{
	MessageInfoTypeDef<T> Msg ={0};
	//remove item from queue
	xQueueReceiveFromISR(_MsgQueue, &Msg, NULL);
	//Clear CS Pin if any
	if(Msg.GPIOx != 0){
		HAL_GPIO_WritePin(Msg.GPIOx, Msg.GPIO_PIN, CSn_INACTIVE_PIN_STATE);
	}
	//Set transaction status
	if(Msg.TransactionStatus != 0){
		*Msg.TransactionStatus = HAL_OK;
	}
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
	return __CheckForNextCommRequestAndStart(NULL);
}

template<typename T>
HAL_StatusTypeDef CommBaseClass<T>::CheckIfSameInstance(const T *pIntStruct)
{
	return (this->_Handle->Instance == pIntStruct->Instance)? HAL_OK : HAL_ERROR;
}


//These Interfaces are supported
template class CommBaseClass<SPI_HandleTypeDef>;
template class CommBaseClass<UART_HandleTypeDef>;
template class CommBaseClass<I2C_HandleTypeDef>;
