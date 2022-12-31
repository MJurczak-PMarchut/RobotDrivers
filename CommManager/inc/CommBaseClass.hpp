/*
 * CommInterface.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */
#ifndef COMMMANAGER_COMMINTERFACE_HPP_
#define COMMMANAGER_COMMINTERFACE_HPP_
#include "Configuration.h"
#include <Queue>

template <class T>
class CommBaseClass
{
	public:
		CommBaseClass(T *hint, DMA_HandleTypeDef *hdmaRx);
		HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef<T> *MsgInfo);
		HAL_StatusTypeDef CheckIfSameInstance(const T *pIntStruct)
		{return (this->_Handle->Instance == pIntStruct->Instance)? HAL_OK : HAL_ERROR;};
		HAL_StatusTypeDef MsgReceivedCB(T *hint);

	protected:
		virtual HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef<T> *MsgInfo) = 0;
		HAL_StatusTypeDef __CheckForNextCommRequestAndStart();
		HAL_StatusTypeDef __CheckAndSetCSPinsGeneric(MessageInfoTypeDef<T> *MsgInfo);

		T *_Handle;
		CommModeTypeDef _commType;
		DMA_HandleTypeDef *hdmaRx;

	private:
		uint16_t GPIO_PIN;
		GPIO_TypeDef *GPIOx;
		std::queue<MessageInfoTypeDef<T>> _MsgQueue;
};

#endif /* COMMMANAGER_COMMINTERFACE_HPP_ */
