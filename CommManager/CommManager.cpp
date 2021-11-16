/*
 * CommManager.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */
#include "CommManager.hpp"
#include "vector"

#ifndef MAX_MESSAGE_NO_IN_QUEUE
#define MAX_MESSAGE_NO_IN_QUEUE 5
#endif

#define VECTOR_NOT_FOUND 0xFF

CommManager::CommManager()
{

}
#ifdef I2C_USES_DMA
HAL_StatusTypeDef CommManager::AttachCommInt(I2C_HandleTypeDef *hi2c, DMA_HandleTypeDef *hdma)
#else
HAL_StatusTypeDef CommManager::AttachCommInt(I2C_HandleTypeDef *hi2c)
#endif
{
	CommQueue<I2C_HandleTypeDef*> Init;
	for(CommQueue<I2C_HandleTypeDef*> handleQueueVect : this->__hi2cQueueVect)
	{
		if(hi2c == handleQueueVect.handle)
		{
			return HAL_ERROR;
		}
	}
	Init.handle = hi2c;
#ifdef I2C_USES_DMA
	Init.hdma = hdma;
#endif
	this->__hi2cQueueVect.push_back(Init);
	return HAL_OK;
}
#ifdef SPI_USES_DMA
HAL_StatusTypeDef CommManager::AttachCommInt(SPI_HandleTypeDef *hspi, DMA_HandleTypeDef *hdma)
#else
HAL_StatusTypeDef CommManager::AttachCommInt(SPI_HandleTypeDef *hspi)
#endif
{
	CommQueue<SPI_HandleTypeDef*> Init;
	for(CommQueue<SPI_HandleTypeDef*> handleQueueVect : this->__hspiQueueVect)
	{
		if(hspi == handleQueueVect.handle)
		{
			return HAL_ERROR;
		}
	}
	Init.handle = hspi;
#ifdef SPI_USES_DMA
	Init.hdma = hdma;
#endif
	this->__hspiQueueVect.push_back(Init);
	return HAL_OK;
}
#ifdef UART_USES_DMA
HAL_StatusTypeDef CommManager::AttachCommInt(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
#else
HAL_StatusTypeDef CommManager::AttachCommInt(UART_HandleTypeDef *huart)
#endif
{
	CommQueue<UART_HandleTypeDef*> Init;
	for(CommQueue<UART_HandleTypeDef*> handleQueueVect : this->__huartQueueVect)
	{
		if(huart == handleQueueVect.handle)
		{
			return HAL_ERROR;
		}
	}
	Init.handle = huart;
#ifdef UART_USES_DMA
	Init.hdma = hdma;
#endif
	this->__huartQueueVect.push_back(Init);
	return HAL_OK;
}

HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef *MsgInfo)
{
	uint8_t VectorIndex;
	VectorIndex = this->__CheckIfCommIntIsAttachedAndHasFreeSpace(&MsgInfo->uCommInt, MsgInfo->eCommType);
	if(VectorIndex != VECTOR_NOT_FOUND)
	{
		return HAL_ERROR;
	}
	else
	{
		switch(MsgInfo->eCommType)
		{
			case COMM_INT_SPI_TXRX:
			{
				if(this->__hspiQueueVect[VectorIndex].MsgInfo.size() > 0)
				{
					this->__hspiQueueVect[VectorIndex].MsgInfo.push(*MsgInfo); //Queue not empty, push message back
				}
				return this->__CheckIfFreeAndSendRecv(MsgInfo, VectorIndex);
			}
				break;
			case COMM_INT_SPI_RX:
			{
				if(this->__hspiQueueVect[VectorIndex].MsgInfo.size() > 0)
				{
					this->__hspiQueueVect[VectorIndex].MsgInfo.push(*MsgInfo); //Queue not empty, push message back
				}
				return this->__CheckIfFreeAndSendRecv(MsgInfo, VectorIndex);
			}
				break;
			case COMM_INT_I2C_TX:
			{
				if(this->__hi2cQueueVect[VectorIndex].MsgInfo.size() > 0)
				{
					this->__hi2cQueueVect[VectorIndex].MsgInfo.push(*MsgInfo); //Queue not empty, push message back
				}
				return this->__CheckIfFreeAndSendRecv(MsgInfo, VectorIndex);
			}
				break;
			case COMM_INT_I2C_RX:
			{
				if(this->__hi2cQueueVect[VectorIndex].MsgInfo.size() > 0)
				{
					this->__hi2cQueueVect[VectorIndex].MsgInfo.push(*MsgInfo); //Queue not empty, push message back
				}
				return this->__CheckIfFreeAndSendRecv(MsgInfo, VectorIndex);
			}

				break;
			case COMM_INT_UART_TX:
			{
				if(this->__huartQueueVect[VectorIndex].MsgInfo.size() > 0)
				{
					this->__huartQueueVect[VectorIndex].MsgInfo.push(*MsgInfo); //Queue not empty, push message back
				}
				return this->__CheckIfFreeAndSendRecv(MsgInfo, VectorIndex);
			}
				break;
			case COMM_INT_UART_RX:
			{
				if(this->__huartQueueVect[VectorIndex].MsgRx.size() > 0)
				{
					this->__huartQueueVect[VectorIndex].MsgRx.push(*MsgInfo); //Queue not empty, push message back
				}
				return this->__CheckIfFreeAndSendRecv(MsgInfo, VectorIndex);
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
			for(u8Iter = 0; u8Iter < this->__hspiQueueVect.size(); u8Iter++)
			{
				if((uCommInt->hspi->Instance == __hspiQueueVect[u8Iter].handle->Instance) && (__hspiQueueVect[u8Iter].MsgInfo.size() < MAX_MESSAGE_NO_IN_QUEUE))
				{
					return u8Iter;
				}
			}
			return VECTOR_NOT_FOUND;
		}
			break;
		case COMM_INT_I2C_TX:
		case COMM_INT_I2C_RX:
		{
			for(u8Iter = 0; u8Iter < this->__hi2cQueueVect.size(); u8Iter++)
			{
				if((uCommInt->hi2c->Instance == __hi2cQueueVect[u8Iter].handle->Instance) && (__hi2cQueueVect[u8Iter].MsgInfo.size() < MAX_MESSAGE_NO_IN_QUEUE))
				{
					return u8Iter;
				}
			}
			return VECTOR_NOT_FOUND;
		}
			break;
		case COMM_INT_UART_TX:
		{
			for(u8Iter = 0; u8Iter < this->__huartQueueVect.size(); u8Iter++)
			{
				if((uCommInt->huart->Instance == __huartQueueVect[u8Iter].handle->Instance) && (__huartQueueVect[u8Iter].MsgInfo.size() < MAX_MESSAGE_NO_IN_QUEUE))
				{
					return u8Iter;
				}
			}
			return VECTOR_NOT_FOUND;
		}
		case COMM_INT_UART_RX:
		{
			for(u8Iter = 0; u8Iter < this->__huartQueueVect.size(); u8Iter++)
			{
				if((uCommInt->huart->Instance == __huartQueueVect[u8Iter].handle->Instance) && (__huartQueueVect[u8Iter].MsgRx.size() < MAX_MESSAGE_NO_IN_QUEUE))
				{
					return u8Iter;
				}
			}
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

HAL_StatusTypeDef CommManager::MsgReceivedCB(UART_HandleTypeDef *huart)
{
	uint8_t u8Iter;
	MessageInfoTypeDef Msg;
	for(u8Iter = 0; u8Iter < this->__hspiQueueVect.size(); u8Iter++)
	{
		if(huart->Instance == __huartQueueVect[u8Iter].handle->Instance)
		{
			if(Msg.eCommType == COMM_INT_UART_TX)
			{
				Msg = this->__huartQueueVect[u8Iter].MsgInfo.front();
				if(Msg.pTxCompletedCB != 0)
				{
					Msg.pTxCompletedCB(&Msg);
				}
				this->__huartQueueVect[u8Iter].MsgInfo.pop();
			}
			else if (Msg.eCommType == COMM_INT_UART_RX)
			{
				Msg = this->__huartQueueVect[u8Iter].MsgRx.front();
				if(Msg.pRxCompletedCB != 0)
				{
					Msg.pRxCompletedCB(&Msg);
				}
				this->__huartQueueVect[u8Iter].MsgRx.pop();
			}
			 return HAL_OK;
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::MsgReceivedCB(SPI_HandleTypeDef *hspi)
{
	uint8_t u8Iter;
	MessageInfoTypeDef Msg;
	for(u8Iter = 0; u8Iter < this->__hspiQueueVect.size(); u8Iter++)
	{
		if(hspi->Instance == __hspiQueueVect[u8Iter].handle->Instance)
		{
			Msg = this->__hspiQueueVect[u8Iter].MsgInfo.front();
			if(Msg.eCommType == COMM_INT_SPI_TXRX)
			{
				if(Msg.pTxCompletedCB != 0)
				{
					Msg.pTxCompletedCB(&Msg);
				}
				if(Msg.pRxCompletedCB != 0)
				{
					Msg.pRxCompletedCB(&Msg);
				}
			}
			else if (Msg.eCommType == COMM_INT_SPI_RX)
			{
				if(Msg.pRxCompletedCB != 0)
				{
					Msg.pRxCompletedCB(&Msg);
				}
			}
			 this->__hspiQueueVect[u8Iter].MsgInfo.pop();
			 return HAL_OK;
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::MsgReceivedCB(I2C_HandleTypeDef *hi2c)
{
	//Find message
	uint8_t u8Iter;
	MessageInfoTypeDef Msg;
	//Find peripheral
	for(u8Iter = 0; u8Iter < this->__hi2cQueueVect.size(); u8Iter++)
	{
		if(hi2c->Instance == __hi2cQueueVect[u8Iter].handle->Instance)
		{
			//Call CB functions if any
			Msg = this->__hi2cQueueVect[u8Iter].MsgInfo.front();
			if(Msg.eCommType == COMM_INT_I2C_RX)
			{
				if(Msg.pTxCompletedCB != 0)
				{
					Msg.pTxCompletedCB(&Msg);
				}
			}
			else if (Msg.eCommType == COMM_INT_I2C_TX)
			{
				if(Msg.pRxCompletedCB != 0)
				{
					Msg.pRxCompletedCB(&Msg);
				}
			}
			//remove item from queue
			 this->__hi2cQueueVect[u8Iter].MsgInfo.pop();
			 return HAL_OK;
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::CheckForNextCommRequestAndStart(I2C_HandleTypeDef *hi2c)
{
	uint8_t VectorIndex;
	MessageInfoTypeDef MsgInfo;
	//Find peripheral
	for(VectorIndex = 0; VectorIndex < this->__hi2cQueueVect.size(); VectorIndex++)
	{
		if(hi2c->Instance == __hi2cQueueVect[VectorIndex].handle->Instance)
		{
			//Check if there are messages in queue
			if(__hi2cQueueVect[VectorIndex].MsgInfo.size() > 1)
			{
				MsgInfo = __hi2cQueueVect[VectorIndex].MsgInfo.front();
				//send message
				return this->__CheckIfFreeAndSendRecv(&MsgInfo, VectorIndex);
			}
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::__CheckIfFreeAndSendRecv(MessageInfoTypeDef *MsgInfo, uint8_t VectorIndex)
{
#if defined(SPI_USES_DMA) or defined(I2C_USES_DMA) or defined(UART_USES_DMA)
	HAL_StatusTypeDef ret;
#endif
	switch(MsgInfo->eCommType)
	{
		case COMM_INT_I2C_RX:
		{
#ifdef I2C_USES_DMA
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Master_Receive_DMA(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__hi2cQueueVect[VectorIndex].hdma, DMA_IT_HT);
				return ret;
			}
#elif defined I2C_USES_IT
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				return HAL_I2C_Master_Receive_IT(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined I2C_USES_WAIT
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				return HAL_I2C_Master_Receive(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
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
				ret = HAL_I2C_Master_Transmit_DMA(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__hi2cQueueVect[VectorIndex].hdma, DMA_IT_HT);
				return ret;
			}
#elif defined I2C_USES_IT
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
				return HAL_I2C_Master_Transmit_IT(MsgInfo->uCommInt.hi2c,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined I2C_USES_WAIT
			if(MsgInfo->uCommInt.hi2c->State == HAL_I2C_STATE_READY)
			{
				//Queue empty and peripheral ready, send message
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
				ret = HAL_SPI_TransmitReceive_DMA(MsgInfo->uCommInt.hspi, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__hspiQueueVect[VectorIndex].hdma, DMA_IT_HT);
				return ret;
			}
#elif defined SPI_USES_IT
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				return HAL_SPI_TransmitReceive_IT(MsgInfo->uCommInt.hspi, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined SPI_USES_WAIT
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				return HAL_SPI_TransmitReceive(MsgInfo->uCommInt.hspi, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len, COMMUNICATION_TIMEOUT);
			}
#endif
		}
		break;
		case COMM_INT_SPI_RX:
		{
#ifdef SPI_USES_DMA
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Receive_DMA(MsgInfo->uCommInt.hspi, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__hspiQueueVect[VectorIndex].hdma, DMA_IT_HT);
				return ret;
			}
#elif defined SPI_USES_IT
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				return HAL_SPI_Receive_IT(MsgInfo->uCommInt.hspi, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined SPI_USES_WAIT
			if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				return HAL_SPI_Receive_IT(MsgInfo->uCommInt.hspi, MsgInfo->pRxData, MsgInfo->len, COMMUNICATION_TIMEOUT);
			}
#endif
		}
		break;
		break;
	case COMM_INT_UART_TX:
	{
#ifdef UART_USES_DMA
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_UART_Transmit_DMA(MsgInfo->uCommInt.huart, MsgInfo->pTxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__huartQueueVect[VectorIndex].hdma, DMA_IT_HT);
				return ret;
			}
#elif defined UART_USES_IT
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				return HAL_UART_Transmit_DMA(MsgInfo->uCommInt.huart, MsgInfo->pTxData, MsgInfo->len);
			}
#elif defined UART_USES_WAIT
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				return HAL_UART_Transmit_DMA(MsgInfo->uCommInt.huart, MsgInfo->pTxData, MsgInfo->len, COMMUNICATION_TIMEOUT);
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
				ret = HAL_UART_Receive_DMA(MsgInfo->uCommInt.huart, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(this->__huartQueueVect[VectorIndex].hdma, DMA_IT_HT);
				return ret;
			}
#elif defined UART_USES_IT
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				return HAL_UART_Receive_IT(MsgInfo->uCommInt.huart, MsgInfo->pRxData, MsgInfo->len);
			}
#elif defined UART_USES_WAIT
			if((MsgInfo->uCommInt.huart->gState == HAL_UART_STATE_READY))
			{
				//Queue empty and peripheral ready, send message
				return HAL_UART_Receive_IT(MsgInfo->uCommInt.huart, MsgInfo->pRxData, MsgInfo->len, COMMUNICATION_TIMEOUT);
			}
#endif
	}
		break;
	default:
	{
		return HAL_ERROR;
	}
	}
	return HAL_OK;
}
