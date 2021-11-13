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
	for(I2C_HandleTypeDef *hi2cptr : this->__hi2cVect)
	{
		if(hi2cptr == hi2c)
		{
			return HAL_ERROR;
		}
	}
	this->__hi2cVect.push_back(hi2c);
	return HAL_OK;
}

HAL_StatusTypeDef CommManager::AttachCommInt(SPI_HandleTypeDef *hspi)
{
	for(SPI_HandleTypeDef *hspiptr : this->__hspiVect)
	{
		if(hspiptr == hspi)
		{
			return HAL_ERROR;
		}
	}
	this->__hspiVect.push_back(hspi);
	return HAL_OK;
}

HAL_StatusTypeDef CommManager::AttachCommInt(UART_HandleTypeDef *huart)
{
	for(UART_HandleTypeDef *huartptr : this->__huartVect)
	{
		if(huartptr == huart)
		{
			return HAL_ERROR;
		}
	}
	this->__huartVect.push_back(huart);
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
			for(SPI_HandleTypeDef *hspiptr : this->__hspiVect)
			{
				if(uCommInt->hspi == hspiptr)
				{
					return HAL_OK;
				}
			}
		}
			break;
		case COMM_INT_I2C_TXRX:
		case COMM_INT_I2C_RX:
		{
			for(I2C_HandleTypeDef *hi2cptr : this->__hi2cVect)
			{
				if(uCommInt->hi2c == hi2cptr)
				{
					return HAL_OK;
				}
			}
		}
			break;
		case COMM_INT_UART_TXRX:
		case COMM_INT_UART_RX:
		{
			for(UART_HandleTypeDef *huartptr : this->__huartVect)
			{
				if(uCommInt->huart == huartptr)
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

