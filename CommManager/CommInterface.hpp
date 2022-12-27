/*
 * CommInterface.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */
#ifndef COMMMANAGER_COMMINTERFACE_HPP_
#define COMMMANAGER_COMMINTERFACE_HPP_
#include "Configuration.h"

template <typename T>
class CommInterface
{
	public:

		virtual HAL_StatusTypeDef AttachCommInt(T *pIntStruct) = 0;
		virtual HAL_StatusTypeDef AttachCommInt(T *pIntStruct, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx){return HAL_ERROR;};
		virtual HAL_StatusTypeDef AttachCommInt(T *pIntStruct, DMA_HandleTypeDef *hdma){return HAL_ERROR;};
		virtual HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef *MsgInfo) = 0;
		virtual void CheckForNextCommRequestAndStart(void) = 0;
		virtual HAL_StatusTypeDef CheckIfSameInstance(const T *pIntStruct) = 0;
		virtual const T* GetInstance(void) = 0;



	protected:
		virtual uint8_t CheckIfCommIntIsAttachedAndHasFreeSpace(CommIntTypeDef eCommIntType) = 0;
	private:
};




#endif /* COMMMANAGER_COMMINTERFACE_HPP_ */
