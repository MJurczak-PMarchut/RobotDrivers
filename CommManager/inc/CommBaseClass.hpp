/*
 * CommInterface.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */
#ifndef COMMMANAGER_COMMINTERFACE_HPP_
#define COMMMANAGER_COMMINTERFACE_HPP_
#include "Configuration.h"
#include "queue.h"

template <class T>
class CommBaseClass
{
	public:
		CommBaseClass(T *hint, DMA_HandleTypeDef *hdmaRx, CommModeTypeDef CommMode, const char* CommName="");
		virtual HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef<T> *MsgInfo);
		HAL_StatusTypeDef CheckIfSameInstance(const T *pIntStruct);
		HAL_StatusTypeDef MsgReceivedCB(T *hint);
		virtual HAL_StatusTypeDef MsgReceivedCB(T *hint, uint16_t len){return HAL_ERROR;};
		virtual HAL_StatusTypeDef MsgReceivedRxCB(T *hint){return HAL_ERROR;};

	protected:
		virtual HAL_StatusTypeDef __CheckIfInterfaceFree(MessageInfoTypeDef<T> *MsgInfo) = 0;
		virtual HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef<T> *MsgInfo) = 0;
		virtual HAL_StatusTypeDef __CheckForNextCommRequestAndStart(MessageInfoTypeDef<T> *MsgInfo);
		HAL_StatusTypeDef __CheckAndSetCSPinsGeneric(MessageInfoTypeDef<T> *MsgInfo);

		T *_Handle;
		DMA_HandleTypeDef *hdmaRx;
		CommModeTypeDef _commType;
		const char* name;
		QueueHandle_t _MsgQueue;

	private:
		uint8_t _xQueueStaticBuffer[MAX_MESSAGE_NO_IN_QUEUE*sizeof(MessageInfoTypeDef<T>)];
		QueueHandle_t  _xStQueue;
		uint16_t GPIO_PIN;
		GPIO_TypeDef *GPIOx;

};

#endif /* COMMMANAGER_COMMINTERFACE_HPP_ */
