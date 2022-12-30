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


HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo)
{
	for(auto instance : _comm_SPI_vect)
	{
		if(instance->CheckIfSameInstance(&MsgInfo->IntHandle) == HAL_OK)
		{
			return instance->PushMessageIntoQueue(MsgInfo);
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo)
{
	for(auto instance : _comm_I2C_vect)
	{
		if(instance->CheckIfSameInstance(&MsgInfo->IntHandle) == HAL_OK)
		{
			return instance->PushMessageIntoQueue(MsgInfo);
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo)
{
	for(auto instance : _comm_UART_vect)
	{
		if(instance->CheckIfSameInstance(&MsgInfo->IntHandle) == HAL_OK)
		{
			return instance->PushMessageIntoQueue(MsgInfo);
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::MsgReceivedCB(UART_HandleTypeDef *huart, uint16_t len)
{
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::MsgReceivedCB(UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef ret_value = HAL_ERROR;
	CommBaseClass<UART_HandleTypeDef>* uart_inst;
	uart_inst = this->_MatchInstance(&_comm_UART_vect, huart);
	if(uart_inst != NULL)
	{
		ret_value = uart_inst->MsgReceivedCB(huart);
	}
	return ret_value;
}

HAL_StatusTypeDef CommManager::MsgReceivedCB(SPI_HandleTypeDef *hspi)
{
	HAL_StatusTypeDef ret_value = HAL_ERROR;
	CommBaseClass<SPI_HandleTypeDef>* spi_inst;
	spi_inst = this->_MatchInstance(&_comm_SPI_vect, hspi);
	if(spi_inst != NULL)
	{
		ret_value = spi_inst->MsgReceivedCB(hspi);
	}
	return ret_value;
}


HAL_StatusTypeDef CommManager::MsgReceivedCB(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef ret_value = HAL_ERROR;
	CommBaseClass<I2C_HandleTypeDef>* i2c_inst;
	i2c_inst = this->_MatchInstance(&_comm_I2C_vect, hi2c);
	if(i2c_inst != NULL)
	{
		ret_value = i2c_inst->MsgReceivedCB(hi2c);
	}
	return ret_value;
}


template<typename T>
HAL_StatusTypeDef CommManager::__CheckAndSetCSPins(MessageInfoTypeDef<T> *MsgInfo, uint8_t VectorIndex)
{
	return HAL_ERROR;
}

template<typename queue, typename T>
HAL_StatusTypeDef CommManager::__CheckAndSetCSPinsGeneric(queue *VectQueue, uint8_t VectorIndex, MessageInfoTypeDef<T> *MsgInfo)
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

template<typename queue, typename T>
CommBaseClass<T>* CommManager::_MatchInstance(queue *VectQueue, T *hint)
{
	for(auto instance : *VectQueue)
	{
		if(instance->CheckIfSameInstance(hint) == HAL_OK)
		{
			return instance;
		}
	}
	return NULL;
}

#endif
