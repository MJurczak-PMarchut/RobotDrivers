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
	CommInterface<T> *Init = _GetObj(hint);
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


#endif /* COMMMANAGER_COMMMANAGERTEMPLATEIMPL_HPP_ */
