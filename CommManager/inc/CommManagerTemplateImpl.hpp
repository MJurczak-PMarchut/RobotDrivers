/*
 * CommManagerTemplateImpl.hpp
 *
 *  Created on: 27 gru 2022
 *      Author: Mateusz
 */

#ifndef COMMMANAGER_COMMMANAGERTEMPLATEIMPL_HPP_
#define COMMMANAGER_COMMMANAGERTEMPLATEIMPL_HPP_


#include "CommI2C.hpp"
#include "CommSPI.hpp"
#include "CommUART.hpp"

template<typename T>
HAL_StatusTypeDef CommManager::AttachCommInt(T *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx)
{
	CommInterface<T>* Init = _GetObj(hint);
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
	this->_PushObjToVect(Init);
	return HAL_OK;
}

template<typename T>
CommInterface<T>* CommManager::_GetObj(T *hint)
{
	if(std::is_same_v<T, I2C_HandleTypeDef>)
	{
		return reinterpret_cast<CommInterface<T>*>(new CommI2C());
	}
	else if(std::is_same_v<T, SPI_HandleTypeDef>)
	{
		return reinterpret_cast<CommInterface<T>*>(new CommSPI());
	}
	else if(std::is_same_v<T, UART_HandleTypeDef>)
	{
		return reinterpret_cast<CommInterface<T>*>(new CommUART());
	}
	else
	{
		return NULL;
	}
}

template<typename T>
HAL_StatusTypeDef CommManager::_PushObjToVect(T hint)
{
	std::vector<T> *Vect;
	if(std::is_same_v<T, CommInterface<I2C_HandleTypeDef>*>)
	{
		Vect = reinterpret_cast<std::vector<T>*>(&this->_comm_I2C_vect);
	}
	else if(std::is_same_v<T, CommInterface<SPI_HandleTypeDef>*>)
	{
		Vect = reinterpret_cast<std::vector<T>*>(&this->_comm_SPI_vect);
	}
	else if(std::is_same_v<T, CommInterface<UART_HandleTypeDef>*>)
	{
		Vect = reinterpret_cast<std::vector<T>*>(&this->_comm_UART_vect);
	}
	else
	{
		return HAL_ERROR;
	}


	for(auto instance : *Vect)
	{
		if(instance->CheckIfSameInstance(hint->GetInstance()) == HAL_OK)
		{
			return HAL_ERROR;
		}
	}

	Vect->push_back(hint);
	return HAL_ERROR;
}



#endif /* COMMMANAGER_COMMMANAGERTEMPLATEIMPL_HPP_ */
