/*
 * CommManager.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */
#include "CommManager.hpp"

#include "vector"

#ifndef NO_COMMUNICATION_INTERFACE_DEFINED
#ifndef MAX_MESSAGE_NO_IN_QUEUE
#define MAX_MESSAGE_NO_IN_QUEUE 5
#endif


#define VECTOR_NOT_FOUND 0xFF

CommManager::CommManager()
{

}

HAL_StatusTypeDef CommManager::_PushObjToVect(CommInterface<I2C_HandleTypeDef>* hint)
{
	for(auto instance : this->_comm_I2C_vect)
	{
		if(instance->CheckIfSameInstance(hint->GetInstance()) == HAL_OK)
		{
			this->_comm_I2C_vect.push_back(hint);
			return HAL_OK;
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::_PushObjToVect(CommInterface<SPI_HandleTypeDef>* hint)
{
	for(auto instance : this->_comm_SPI_vect)
	{
		if(instance->CheckIfSameInstance(hint->GetInstance()) == HAL_OK)
		{
			this->_comm_SPI_vect.push_back(hint);
			return HAL_OK;
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::_PushObjToVect(CommInterface<UART_HandleTypeDef>* hint)
{
	for(auto instance : this->_comm_UART_vect)
	{
		if(instance->CheckIfSameInstance(hint->GetInstance()) == HAL_OK)
		{
			this->_comm_UART_vect.push_back(hint);
			return HAL_OK;
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef *MsgInfo)
{
	uint8_t VectorIndex;
	VectorIndex = this->__CheckIfCommIntIsAttachedAndHasFreeSpace(&MsgInfo->uCommInt, MsgInfo->eCommType);
	if(VectorIndex == VECTOR_NOT_FOUND)
	{
		return HAL_ERROR;
	}
	else
	{
		if(MsgInfo->TransactionStatus !=0)
		{
			*MsgInfo->TransactionStatus = HAL_BUSY;
		}
		switch(MsgInfo->eCommType)
		{
			case COMM_INT_SPI_TXRX:
			case COMM_INT_SPI_RX:
			{
#if defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT)
				__disable_irq();
				this->__hspiQueueVect[VectorIndex].MsgInfo.push(*MsgInfo); //Queue not empty, push message back
				__enable_irq();
				return this->__CheckForNextCommRequestAndStart(MsgInfo->uCommInt.hspi, &this->__hspiQueueVect);
#endif
			}
				break;
			case COMM_INT_I2C_TX:
			case COMM_INT_I2C_RX:
			case COMM_INT_I2C_MEM_TX:
			case COMM_INT_I2C_MEM_RX:
			{
#if defined(I2C_USES_DMA) or defined(I2C_USES_IT) or defined(I2C_USES_WAIT)
				__disable_irq();
				this->__hi2cQueueVect[VectorIndex].MsgInfo.push(*MsgInfo); //Queue not empty, push message back
				__enable_irq();
				return this->__CheckForNextCommRequestAndStart(MsgInfo->uCommInt.hi2c, &this->__hi2cQueueVect);
#endif
			}
				break;
			case COMM_INT_UART_TX:
			{
#if defined(UART_USES_DMA) or defined(UART_USES_IT)
				__disable_irq();
				this->__huartQueueVect[VectorIndex].MsgInfo.push(*MsgInfo); //Queue not empty, push message back
				__enable_irq();
				return this->__CheckForNextCommRequestAndStart(MsgInfo->uCommInt.huart, &this->__huartQueueVect);
#elif defined(UART_USES_WAIT)
				return this->__CheckForNextCommRequestAndStart(MsgInfo->uCommInt.huart, this->__huartQueueVect);
#endif
			}
				break;
			case COMM_INT_UART_RX:
			{
#if defined(UART_USES_DMA) or defined(UART_USES_IT)
				__disable_irq();
				this->__huartQueueVect[VectorIndex].MsgRx.push(*MsgInfo); //Queue not empty, push message back
				__enable_irq();
				return this->__CheckForNextCommRequestAndStart(MsgInfo->uCommInt.huart, &this->__huartQueueVect);
#elif defined(UART_USES_WAIT)
				return this->__CheckForNextCommRequestAndStart(MsgInfo->uCommInt.huart, this->__huartQueueVect);
#endif
			}
				break;
			default:
			{
				return HAL_ERROR;
			}
		}
	}
	return HAL_ERROR;
}


uint8_t CommManager::__CheckIfCommIntIsAttachedAndHasFreeSpace(CommIntUnionTypeDef *uCommInt, CommIntTypeDef eCommIntType)
{
	uint8_t u8Iter;
	switch(eCommIntType)
	{
		case COMM_INT_SPI_TXRX:
		case COMM_INT_SPI_RX:
		{
#if defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT)
			for(u8Iter = 0; u8Iter < this->__hspiQueueVect.size(); u8Iter++)
			{
				if((uCommInt->hspi->Instance == __hspiQueueVect[u8Iter].handle->Instance) && (__hspiQueueVect[u8Iter].MsgInfo.size() < MAX_MESSAGE_NO_IN_QUEUE))
				{
					return u8Iter;
				}
			}
#endif
			return VECTOR_NOT_FOUND;
		}
			break;
		case COMM_INT_I2C_TX:
		case COMM_INT_I2C_RX:
		case COMM_INT_I2C_MEM_TX:
		case COMM_INT_I2C_MEM_RX:
		{
#if defined(I2C_USES_DMA) or defined(I2C_USES_IT) or defined(I2C_USES_WAIT)
			for(u8Iter = 0; u8Iter < this->__hi2cQueueVect.size(); u8Iter++)
			{
				if((uCommInt->hi2c->Instance == __hi2cQueueVect[u8Iter].handle->Instance) && (__hi2cQueueVect[u8Iter].MsgInfo.size() < MAX_MESSAGE_NO_IN_QUEUE))
				{
					return u8Iter;
				}
			}
#endif
			return VECTOR_NOT_FOUND;
		}
			break;
		case COMM_INT_UART_TX:
		{
#if defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)
			for(u8Iter = 0; u8Iter < this->__huartQueueVect.size(); u8Iter++)
			{
				if((uCommInt->huart->Instance == __huartQueueVect[u8Iter].handle->Instance) && (__huartQueueVect[u8Iter].MsgInfo.size() < MAX_MESSAGE_NO_IN_QUEUE))
				{
					return u8Iter;
				}
			}
#endif
			return VECTOR_NOT_FOUND;
		}
		case COMM_INT_UART_RX:
		{
#if defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)
			for(u8Iter = 0; u8Iter < this->__huartQueueVect.size(); u8Iter++)
			{
				if((uCommInt->huart->Instance == __huartQueueVect[u8Iter].handle->Instance) && (__huartQueueVect[u8Iter].MsgRx.size() < MAX_MESSAGE_NO_IN_QUEUE))
				{
					return u8Iter;
				}
			}
#endif
			return VECTOR_NOT_FOUND;
		}
			break;
		default:
		{
			return VECTOR_NOT_FOUND;
		}
	}
	return VECTOR_NOT_FOUND;
}


#if defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)
HAL_StatusTypeDef CommManager::MsgReceivedCB(UART_HandleTypeDef *huart, uint16_t len)
{
	uint8_t u8Iter;
	MessageInfoTypeDef Msg ;
	for(u8Iter = 0; u8Iter < this->__huartQueueVect.size(); u8Iter++)
	{
		if(huart->Instance == __huartQueueVect[u8Iter].handle->Instance)
		{
			Msg = this->__huartQueueVect[u8Iter].MsgRx.front();
			this->__huartQueueVect[u8Iter].MsgRx.pop();
			if (Msg.eCommType == COMM_INT_UART_RX)
			{
				if(Msg.pRxCompletedCB != 0)
				{
					Msg.len = len;
					Msg.pRxCompletedCB(&Msg);
				}
			}
			 return  __CheckForNextCommRequestAndStart(huart, &this->__huartQueueVect);
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::MsgReceivedCB(UART_HandleTypeDef *huart)
{
	return __MsgReceivedCB(huart, &this->__huartQueueVect);
}
#endif

#if defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT)
HAL_StatusTypeDef CommManager::MsgReceivedCB(SPI_HandleTypeDef *hspi)
{
	return __MsgReceivedCB(hspi, &this->__hspiQueueVect);
}
#endif


#if defined(I2C_USES_DMA) or defined(I2C_USES_IT) or defined(I2C_USES_WAIT)
HAL_StatusTypeDef CommManager::MsgReceivedCB(I2C_HandleTypeDef *hi2c)
{
	return __MsgReceivedCB(hi2c, &this->__hi2cQueueVect);
}
#endif

template<typename Handle, typename QueueVectTD>
HAL_StatusTypeDef CommManager::__MsgReceivedCB(Handle *IntHandle, QueueVectTD *Queue)
{
	//Find message
	uint8_t u8Iter;
	MessageInfoTypeDef Msg;
	//Find peripheral
	for(u8Iter = 0; u8Iter < (*Queue).size(); u8Iter++)
	{
		if(IntHandle->Instance == (*Queue)[u8Iter].handle->Instance)
		{
			//Call CB functions if any
			Msg = (*Queue)[u8Iter].MsgInfo.front();
			//remove item from queue
			(*Queue)[u8Iter].MsgInfo.pop();
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
				case COMM_INT_I2C_RX:
				case COMM_INT_SPI_RX:
				case COMM_INT_UART_RX:
				case COMM_INT_I2C_MEM_RX:
				{
					if(Msg.pRxCompletedCB != 0)
						{
							Msg.pRxCompletedCB(&Msg);
						}
				}
					break;
				case COMM_INT_I2C_TX:
				case COMM_INT_UART_TX:
				case COMM_INT_I2C_MEM_TX:
				{
					if(Msg.pTxCompletedCB != 0)
						{
							Msg.pTxCompletedCB(&Msg);
						}
				}
					break;
				case COMM_INT_SPI_TXRX:
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
			 return __CheckForNextCommRequestAndStart(IntHandle, Queue);
		}
	}
	return HAL_ERROR;
}

template<typename Handle, typename QueueVectTD>
HAL_StatusTypeDef CommManager::__CheckForNextCommRequestAndStart(Handle *IntHandle, QueueVectTD *Queue)
{
	uint8_t VectorIndex;
	MessageInfoTypeDef MsgInfo;
	HAL_StatusTypeDef ret = HAL_ERROR;

	//Find peripheral
	for(VectorIndex = 0; VectorIndex < Queue->size(); VectorIndex++)
	{
		if(IntHandle->Instance == (*Queue)[VectorIndex].handle->Instance)
		{
			//Check if there are messages in queue
			if((*Queue)[VectorIndex].MsgInfo.size() > MAX_MESSAGE_NO_IN_QUEUE)
			{
				if((*Queue)[VectorIndex].MsgInfo.empty())
				{
					return HAL_OK;
				}
			}
			else if((*Queue)[VectorIndex].MsgInfo.size() > 0)
			{
//				auto size = (*Queue)[VectorIndex].MsgInfo.size();
				MsgInfo =(*Queue)[VectorIndex].MsgInfo.front();
				//send message
				ret = this->__CheckIfFreeAndSendRecv(&MsgInfo, VectorIndex);
				if(ret == HAL_ERROR)
				{
					//Unable to send
					(*Queue)[VectorIndex].MsgInfo.pop();
				}
			}
		}
	}
	return ret;
}

HAL_StatusTypeDef CommManager::__CheckIfFreeAndSendRecv(MessageInfoTypeDef *MsgInfo, uint8_t VectorIndex)
{
#if defined(SPI_USES_DMA) or defined(I2C_USES_DMA) or defined(UART_USES_DMA)
	HAL_StatusTypeDef ret =HAL_BUSY;
#endif
	// Set Pins
	switch(MsgInfo->eCommType)
	{
		case COMM_INT_I2C_RX:
		{
#ifdef I2C_USES_DMA
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_I2C_Master_Receive_DMA(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__hi2cQueueVect[VectorIndex].hdmaRx, DMA_IT_HT);
				return ret;
			}
#elif defined I2C_USES_IT
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_I2C_Master_Receive_IT(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined I2C_USES_WAIT
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				return HAL_I2C_Master_Receive(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
			}
#endif
		}
			break;
		case COMM_INT_I2C_MEM_RX:
		{
#ifdef I2C_USES_DMA
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_I2C_Mem_Read_DMA(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__hi2cQueueVect[VectorIndex].hdmaRx, DMA_IT_HT);
				return ret;
			}
#elif defined I2C_USES_IT
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_I2C_Mem_Read_IT(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined I2C_USES_WAIT
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				return HAL_I2C_Mem_Read(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pRxData, MsgInfo->len);
			}
#endif
		}
			break;
		case COMM_INT_I2C_MEM_TX:
				{
		#ifdef I2C_USES_DMA
					if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
					{
						//Queue empty and peripheral ready, send message
						__CheckAndSetCSPins(MsgInfo, VectorIndex);
						ret = HAL_I2C_Mem_Write_DMA(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pRxData, MsgInfo->len);
						__HAL_DMA_DISABLE_IT(this->__hi2cQueueVect[VectorIndex].hdmaRx, DMA_IT_HT);
						return ret;
					}
		#elif defined I2C_USES_IT
					if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
					{
						//Queue empty and peripheral ready, send message
						__CheckAndSetCSPins(MsgInfo, VectorIndex);
						ret = HAL_I2C_Mem_Write_IT(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pTxData, MsgInfo->len);
					}
		#elif defined I2C_USES_WAIT
					if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
					{
						//Queue empty and peripheral ready, send message
						__CheckAndSetCSPins(MsgInfo, VectorIndex);
						return HAL_I2C_Mem_Write(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pRxData, MsgInfo->len);
					}
		#endif
				}
					break;
		case COMM_INT_I2C_TX:
		{
#ifdef I2C_USES_DMA
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_I2C_Master_Transmit_DMA(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__hi2cQueueVect[VectorIndex].hdmaTx, DMA_IT_HT);
				return ret;
			}
#elif defined I2C_USES_IT
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_I2C_Master_Transmit_IT(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined I2C_USES_WAIT
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				return HAL_I2C_Master_Transmit(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
			}
#endif
		}
			break;
		case COMM_INT_SPI_TXRX:
		{
#ifdef SPI_USES_DMA
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_SPI_TransmitReceive_DMA(MsgInfo->uCommInt.hspi, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__hspiQueueVect[VectorIndex].hdmaTx, DMA_IT_HT);
				return ret;
			}
#elif defined SPI_USES_IT
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_SPI_TransmitReceive_IT(MsgInfo->uCommInt.hspi, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined SPI_USES_WAIT
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				return HAL_SPI_TransmitReceive(MsgInfo->uCommInt.hspi, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len, COMMUNICATION_TIMEOUT);
			}
#else
			return HAL_ERROR;
#endif
		}
			break;
		case COMM_INT_SPI_RX:
		{
#ifdef SPI_USES_DMA
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_SPI_Receive_DMA(MsgInfo->uCommInt.hspi, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__hspiQueueVect[VectorIndex].hdmaRx, DMA_IT_HT);
				return ret;
			}
#elif defined SPI_USES_IT
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_SPI_Receive_IT(MsgInfo->uCommInt.hspi, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined SPI_USES_WAIT
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				return HAL_SPI_Receive_IT(MsgInfo->uCommInt.hspi, MsgInfo->pRxData, MsgInfo->len, COMMUNICATION_TIMEOUT);
			}
#else
			return HAL_ERROR;
#endif
		}
			break;
		case COMM_INT_UART_TX:
		{
#ifdef UART_USES_DMA
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_UART_Transmit_DMA(MsgInfo->uCommInt.huart, MsgInfo->pTxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__huartQueueVect[VectorIndex].hdmaTx, DMA_IT_HT);
			}
#elif defined UART_USES_IT
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				return HAL_UART_Transmit_IT(MsgInfo->uCommInt.huart, MsgInfo->pTxData, MsgInfo->len);
			}
#elif defined UART_USES_WAIT
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				return HAL_UART_Transmit(MsgInfo->uCommInt.huart, MsgInfo->pTxData, MsgInfo->len, COMMUNICATION_TIMEOUT);
			}
#endif
		}
			break;
		case COMM_INT_UART_RX:
		{
#ifdef UART_USES_DMA
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				ret = HAL_UARTEx_ReceiveToIdle_DMA(MsgInfo->uCommInt.huart, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__huartQueueVect[VectorIndex].hdmaRx, DMA_IT_HT);
			}
#elif defined UART_USES_IT
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				return HAL_UARTEx_ReceiveToIdle_IT(MsgInfo->uCommInt.huart, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined UART_USES_WAIT
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				__CheckAndSetCSPins(MsgInfo, VectorIndex);
				return HAL_UART_Receive(MsgInfo->uCommInt.huart, MsgInfo->pRxData, MsgInfo->len, COMMUNICATION_TIMEOUT);
			}
#endif
		}
			break;
		default:
		{
			return HAL_ERROR;
		}
	}
	return ret;
}

HAL_StatusTypeDef CommManager::__CheckAndSetCSPins(MessageInfoTypeDef *MsgInfo, uint8_t VectorIndex)
{

	switch(MsgInfo->eCommType)
	{
		case COMM_INT_SPI_TXRX:
		case COMM_INT_SPI_RX:
		{
#if defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT)
			return __CheckAndSetCSPinsGeneric(&this->__hspiQueueVect, VectorIndex, MsgInfo);
#else
			return HAL_ERROR;
#endif
		}
			break;
		case COMM_INT_I2C_TX:
		case COMM_INT_I2C_RX:
		{
#if defined(I2C_USES_DMA) or defined(I2C_USES_IT) or defined(I2C_USES_WAIT)
			return __CheckAndSetCSPinsGeneric(&this->__hi2cQueueVect, VectorIndex, MsgInfo);
#else
			return HAL_ERROR;
#endif
		}
			break;
		case COMM_INT_UART_TX:
		case COMM_INT_UART_RX:
		{
#if defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)
			return __CheckAndSetCSPinsGeneric(&this->__huartQueueVect, VectorIndex, MsgInfo);
#else
			return HAL_ERROR;
#endif
		}
			break;

		default:
		{
			return HAL_ERROR;
		}
	}
	return HAL_ERROR;
}

template<typename queue>
HAL_StatusTypeDef CommManager::__CheckAndSetCSPinsGeneric(queue *VectQueue, uint8_t VectorIndex, MessageInfoTypeDef *MsgInfo)
{

		//Check if pin is already set
		if(((*VectQueue)[VectorIndex].GPIO_PIN == MsgInfo->GPIO_PIN) && ((*VectQueue)[VectorIndex].GPIOx == MsgInfo->GPIOx))
		{
			//if so return
			return HAL_OK;
		}
		else
		{
			//if not reset pin states and set new ones
			HAL_GPIO_WritePin((*VectQueue)[VectorIndex].GPIOx, (*VectQueue)[VectorIndex].GPIO_PIN, CSn_INACTIVE_PIN_STATE);
			if(MsgInfo->GPIOx == 0)
			{
				return HAL_OK;
			}
			HAL_GPIO_WritePin(MsgInfo->GPIOx, MsgInfo->GPIO_PIN, CSn_ACTIVE_PIN_STATE);
			return HAL_OK;
		}

}

#endif
