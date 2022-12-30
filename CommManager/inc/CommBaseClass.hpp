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

template <typename T>
struct CommQueue{
	T handle;
#if defined(SPI_USES_DMA) or defined(I2C_USES_DMA) or defined(UART_USES_DMA)
	DMA_HandleTypeDef *hdmaRx;
	DMA_HandleTypeDef *hdmaTx;
#endif
	std::queue<MessageInfoTypeDef<T>> MsgInfo;
	std::queue<MessageInfoTypeDef<T>> MsgRx;
	uint16_t GPIO_PIN;
	GPIO_TypeDef *GPIOx;
};

template <typename T>
class CommBaseClass
{
	public:

		virtual HAL_StatusTypeDef AttachCommInt(T *pIntStruct, CommModeTypeDef commType = COMM_DUMMY) = 0;
		virtual HAL_StatusTypeDef AttachCommInt(T *pIntStruct, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx, CommModeTypeDef commType = COMM_DUMMY){return HAL_ERROR;};
		virtual HAL_StatusTypeDef AttachCommInt(T *pIntStruct, DMA_HandleTypeDef *hdma, CommModeTypeDef commType = COMM_DUMMY){return HAL_ERROR;};
		virtual HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef<T> *MsgInfo) = 0;
		virtual void CheckForNextCommRequestAndStart(void) = 0;
		virtual HAL_StatusTypeDef CheckIfSameInstance(const T *pIntStruct) = 0;
		virtual const T* GetInstance(void) = 0;

		HAL_StatusTypeDef MsgReceivedCB(T *hint);

	protected:
		virtual HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef<T> *MsgInfo) {return HAL_ERROR;};

		HAL_StatusTypeDef __CheckForNextCommRequestAndStart();
		HAL_StatusTypeDef __CheckAndSetCSPinsGeneric(MessageInfoTypeDef<T> *MsgInfo);
		std::queue<MessageInfoTypeDef<T>> _MsgQueue;
		T *_Handle;
		CommModeTypeDef _commType;
		DMA_HandleTypeDef *hdmaRx;
		DMA_HandleTypeDef *hdmaTx;
	private:
		uint16_t GPIO_PIN;
		GPIO_TypeDef *GPIOx;

};




#endif /* COMMMANAGER_COMMINTERFACE_HPP_ */
