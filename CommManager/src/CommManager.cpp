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

#include "CommI2C.hpp"
#include "CommSPI.hpp"
#include "CommUART.hpp"



#define VECTOR_NOT_FOUND 0xFF

CommManager::CommManager()
{

}

template<typename T>
HAL_StatusTypeDef CommManager::AttachCommInt(T *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx)
{
	CommBaseClass<T>* Init = NULL;
	std::vector<CommBaseClass<T>*>* vect = _GetVector(hint);
	if(this->_MatchInstance(vect, hint) != NULL)
	{
		return HAL_ERROR;
	}
	// ToDo check if args are correct
	Init = _GetObj(hint, hdmaRx, hdmaTx);
	if(Init == NULL)
	{
		return HAL_ERROR;
	}
	if(hdmaRx == NULL){
		if(Init->AttachCommInt(hint) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	else if (hdmaTx == NULL)
	{
		if(Init->AttachCommInt(hint, hdmaRx) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	else
	{
		if(Init->AttachCommInt(hint, hdmaRx, hdmaTx) != HAL_OK)
		{
			return HAL_ERROR;
		}
	}
	vect->push_back(Init);
	return HAL_OK;
}

template<typename T>
HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef<T> *MsgInfo)
{
	std::vector<CommBaseClass<T>*>* vect = _GetVector(&MsgInfo->IntHandle);
	if(vect == NULL)
	{
		return HAL_ERROR;
	}
	for(auto instance : *vect)
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

template<typename T>
HAL_StatusTypeDef CommManager::MsgReceivedCB(T *hint)
{
	HAL_StatusTypeDef ret_value = HAL_ERROR;
	CommBaseClass<T>* handle_inst;
	handle_inst = this->_MatchInstance(_GetVector(hint), hint);
	if(handle_inst != NULL)
	{
		ret_value = handle_inst->MsgReceivedCB(hint);
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

template<>
std::vector<CommBaseClass<I2C_HandleTypeDef>*>* CommManager::_GetVector(I2C_HandleTypeDef *hint)
{
	return &_comm_I2C_vect;
}

template<>
std::vector<CommBaseClass<SPI_HandleTypeDef>*>* CommManager::_GetVector(SPI_HandleTypeDef *hint)
{
	return &_comm_SPI_vect;
}

template<>
std::vector<CommBaseClass<UART_HandleTypeDef>*>* CommManager::_GetVector(UART_HandleTypeDef *hint)
{
	return &_comm_UART_vect;
}

template<>
CommBaseClass<UART_HandleTypeDef>* CommManager::_GetObj(UART_HandleTypeDef *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx)
{
	return new CommUART(hint, hdmaRx, hdmaTx);
}

template<>
CommBaseClass<SPI_HandleTypeDef>* CommManager::_GetObj(SPI_HandleTypeDef *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx)
{
	return new CommSPI(hint, hdmaRx);
}

template<>
CommBaseClass<I2C_HandleTypeDef>* CommManager::_GetObj(I2C_HandleTypeDef *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx)
{
	return new CommI2C(hint, hdmaRx);
}

#endif
