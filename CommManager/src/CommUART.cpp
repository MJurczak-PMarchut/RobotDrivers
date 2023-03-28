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

HAL_StatusTypeDef CommUART::__CheckIfFreeAndSendRecv(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo)
{
	HAL_StatusTypeDef ret =HAL_BUSY;
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
	std::queue<MessageInfoTypeDef<UART_HandleTypeDef>> *pMsgQueue;
	if(MsgInfo->eCommType == COMM_INT_RX){
		pMsgQueue = &this->_RxMsgQueue;
	}
	else{
		pMsgQueue = &this->_MsgQueue;
	}

	if(MsgInfo->TransactionStatus !=0)
	{
		*MsgInfo->TransactionStatus = HAL_BUSY;
	}
	if(MsgInfo->IntHandle->Instance != this->_Handle->Instance)
	{
		return HAL_ERROR;
	}
	if(pMsgQueue->size() >= MAX_MESSAGE_NO_IN_QUEUE)
	{
		return HAL_ERROR;
	}
	__disable_irq();
	pMsgQueue->push(*MsgInfo); //Queue not empty, push message back
	__enable_irq();
	if(MsgInfo->eCommType == COMM_INT_RX){
		return this->__CheckForNextRxCommRequestAndStart();
	}
	else{
		return this->__CheckForNextCommRequestAndStart();
	}

}

HAL_StatusTypeDef CommUART::__CheckForNextRxCommRequestAndStart()
{
	HAL_StatusTypeDef ret = HAL_OK;
	MessageInfoTypeDef<UART_HandleTypeDef> Msg;
	if(_RxMsgQueue.size() > MAX_MESSAGE_NO_IN_QUEUE)
	{
		if(_RxMsgQueue.empty())
		{
			return HAL_OK;
		}
	}
	else if(_RxMsgQueue.size() > 0)
	{
		//				auto size = (*Queue)[VectorIndex].MsgInfo.size();
		Msg =_RxMsgQueue.front();
		//send message
		ret = this->__CheckIfFreeAndSendRecv(&Msg);
		if(ret == HAL_ERROR)
		{
			//Unable to send
			_RxMsgQueue.pop();
		}
	}
	return ret;
}

HAL_StatusTypeDef CommUART::MsgReceivedRxCB(UART_HandleTypeDef *hint)
{
	MessageInfoTypeDef<UART_HandleTypeDef> Msg = _RxMsgQueue.front();
	return this->MsgReceivedCB(hint, Msg.len);
}

HAL_StatusTypeDef CommUART::MsgReceivedCB(UART_HandleTypeDef *hint, uint16_t len)
{
	MessageInfoTypeDef<UART_HandleTypeDef> Msg = _RxMsgQueue.front();
	//remove item from queue
	_RxMsgQueue.pop();
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
		return this->__CheckForNextRxCommRequestAndStart();
	}
	else{
		return this->__CheckForNextCommRequestAndStart();
	}
}

/*
 * CommUART.cpp
 *
 *  Created on: 30 gru 2022
 *      Author: Paulina
 */




