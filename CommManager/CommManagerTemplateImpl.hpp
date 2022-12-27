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
	return HAL_OK;
}




#endif /* COMMMANAGER_COMMMANAGERTEMPLATEIMPL_HPP_ */
