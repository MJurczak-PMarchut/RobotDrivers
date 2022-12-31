/*
 * CommBaseClass.cpp
 *
 *  Created on: 30 gru 2022
 *      Author: Paulina
 */
#include "CommBaseClass.hpp"

template<typename T>
CommBaseClass<T>::CommBaseClass(T *hint, DMA_HandleTypeDef *hdmaRx)
:_Handle{hint}, hdmaRx{hdmaRx}
{}


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
			HAL_GPIO_WritePin(GPIOx, GPIO_PIN, CSn_INACTIVE_PIN_STATE);
			if(MsgInfo->GPIOx == 0)
			{
				return HAL_OK;
			}
			HAL_GPIO_WritePin(MsgInfo->GPIOx, MsgInfo->GPIO_PIN, CSn_ACTIVE_PIN_STATE);
			return HAL_OK;
		}

}

template<class T>
HAL_StatusTypeDef CommBaseClass<T>::__CheckForNextCommRequestAndStart()
{
	HAL_StatusTypeDef ret = HAL_OK;
	MessageInfoTypeDef<T> Msg;
	if(_MsgQueue.size() > MAX_MESSAGE_NO_IN_QUEUE)
	{
		if(_MsgQueue.empty())
		{
			return HAL_OK;
		}
	}
	else if(_MsgQueue.size() > 0)
	{
		//				auto size = (*Queue)[VectorIndex].MsgInfo.size();
		Msg =_MsgQueue.front();
		//send message
		ret = this->__CheckIfFreeAndSendRecv(&Msg);
		if(ret == HAL_ERROR)
		{
			//Unable to send
			_MsgQueue.pop();
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
	if(_MsgQueue.size() >= MAX_MESSAGE_NO_IN_QUEUE)
	{
		return HAL_ERROR;
	}
	__disable_irq();
	this->_MsgQueue.push(*MsgInfo); //Queue not empty, push message back
	__enable_irq();
	return this->__CheckForNextCommRequestAndStart();
}

template<typename T>
HAL_StatusTypeDef CommBaseClass<T>::MsgReceivedCB(T *hint)
{
	MessageInfoTypeDef<T> Msg = _MsgQueue.front();
	//remove item from queue
	_MsgQueue.pop();
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
	return __CheckForNextCommRequestAndStart();
}

//These Interfaces are supported
template class CommBaseClass<SPI_HandleTypeDef>;
template class CommBaseClass<UART_HandleTypeDef>;
template class CommBaseClass<I2C_HandleTypeDef>;
