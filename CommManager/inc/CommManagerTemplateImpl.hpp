/*
 * CommManagerTemplateImpl.hpp
 *
 *  Created on: 31 gru 2022
 *      Author: Paulina
 */

#ifndef COMMMANAGER_COMMMANAGERTEMPLATEIMPL_HPP_
#define COMMMANAGER_COMMMANAGERTEMPLATEIMPL_HPP_

#include "CommManager.hpp"

template<typename T>
HAL_StatusTypeDef CommManager::AttachCommInt(T *hint, CommModeTypeDef CommMode, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx)
{
	CommBaseClass<T>* Init = NULL;
	std::vector<CommBaseClass<T>*>* vect = _GetVector(hint);
	if(this->_MatchInstance(vect, hint) != NULL)
	{
		return HAL_ERROR;
	}
	// ToDo check if args are correct
	Init = _GetObj(hint, hdmaRx, hdmaTx, CommMode);
	if(Init == NULL)
	{
		return HAL_ERROR;
	}
	vect->push_back(Init);
	return HAL_OK;
}

template<class T>
HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef<T> *MsgInfo)
{
	std::vector<CommBaseClass<T>*>* vect = _GetVector(MsgInfo->IntHandle);
	if(vect == NULL)
	{
		return HAL_ERROR;
	}
	for(auto instance : *vect)
	{
		if(instance->CheckIfSameInstance(MsgInfo->IntHandle) == HAL_OK)
		{
			return instance->PushMessageIntoQueue(MsgInfo);

		}
	}
	return HAL_ERROR;
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

#endif /* COMMMANAGER_COMMMANAGERTEMPLATEIMPL_HPP_ */
