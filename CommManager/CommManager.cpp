/*
 * CommManager.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */
#include "CommManager.hpp"
#include "vector"

CommManager::CommManager()
{

}

HAL_StatusTypeDef CommManager::AttachCommInt(I2C_HandleTypeDef *hi2c)
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
	this->__hi2cQueueVect.push_back(Init);
	return HAL_OK;
}

HAL_StatusTypeDef CommManager::AttachCommInt(SPI_HandleTypeDef *hspi)
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
	this->__hspiQueueVect.push_back(Init);
	return HAL_OK;
}

HAL_StatusTypeDef CommManager::AttachCommInt(UART_HandleTypeDef *huart)
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
	this->__huartQueueVect.push_back(Init);
	return HAL_OK;
}

HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef *MsgInfo)
{
	if(this->__CheckIfCommIntIsAttached(&MsgInfo->uCommInt, MsgInfo->eCommType) != HAL_OK)
	{
		return HAL_ERROR;
	}
	else
	{
		switch(MsgInfo->eCommType)
		{
			case COMM_INT_SPI_TXRX:
			{
#ifdef SPI_USES_DMA
				if((MsgInfo->uCommInt.hspi->State == HAL_SPI_STATE_READY))
				{
//TODO Check if queue has free space for a message
				}
#elif defined SPI_USES_IT

#elif defined SPI_USES_WAIT
#error ("SPI_USES_WAIT is not supported")
#endif
			}
				break;
			case COMM_INT_SPI_RX:

				break;
			case COMM_INT_I2C_TXRX:

				break;
			case COMM_INT_I2C_RX:

				break;
			case COMM_INT_UART_TXRX:

				break;
			case COMM_INT_UART_RX:

				break;
			default:
			{
				return HAL_ERROR;
			}
		}
	}
}

HAL_StatusTypeDef CommManager::__CheckIfCommIntIsAttached(CommIntUnionTypeDef *uCommInt, CommIntTypeDef eCommIntType)
{
	switch(eCommIntType)
	{
		case COMM_INT_SPI_TXRX:
		case COMM_INT_SPI_RX:
		{
			for(CommQueue<SPI_HandleTypeDef*> hspiptr : this->__hspiQueueVect)
			{
				if(uCommInt->hspi == hspiptr.handle)
				{
					return HAL_OK;
				}
			}
		}
			break;
		case COMM_INT_I2C_TXRX:
		case COMM_INT_I2C_RX:
		{
			for(CommQueue<I2C_HandleTypeDef*> hi2cptr : this->__hi2cQueueVect)
			{
				if(uCommInt->hi2c == hi2cptr.handle)
				{
					return HAL_OK;
				}
			}
		}
			break;
		case COMM_INT_UART_TXRX:
		case COMM_INT_UART_RX:
		{
			for(CommQueue<UART_HandleTypeDef*> huartptr : this->__huartQueueVect)
			{
				if(uCommInt->huart == huartptr.handle)
				{
					return HAL_OK;
				}
			}
		}
			break;
		default:
		{
			return HAL_ERROR;
		}
	}
	return HAL_ERROR;
}

