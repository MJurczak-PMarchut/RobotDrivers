/*
 * CommManager.hpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#ifndef SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_
#define SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_
#include "Configuration.h"

#ifndef NO_COMMUNICATION_INTERFACE_DEFINED
#include "vector"
#include "queue"
#include <functional>
#include "CommInterface.hpp"

//TODO that is the dumbest way to do it possible, I don't like it, I should probably fix this in the future

template <typename T>
struct CommQueue{
	T handle;
#if defined(SPI_USES_DMA) or defined(I2C_USES_DMA) or defined(UART_USES_DMA)
	DMA_HandleTypeDef *hdmaRx;
	DMA_HandleTypeDef *hdmaTx;
#endif
	std::queue<MessageInfoTypeDef> MsgInfo;
	std::queue<MessageInfoTypeDef> MsgRx;
	uint16_t GPIO_PIN;
	GPIO_TypeDef *GPIOx;
};


class CommManager
{
	public:
		CommManager();
		HAL_StatusTypeDef PushCommRequestIntoQueue(MessageInfoTypeDef *MsgInfo);

#if defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)

		HAL_StatusTypeDef MsgReceivedCB(UART_HandleTypeDef *huart, uint16_t len);
		HAL_StatusTypeDef MsgReceivedCB(UART_HandleTypeDef *huart);
#endif
#if defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT)

		HAL_StatusTypeDef MsgReceivedCB(SPI_HandleTypeDef *hspi);
#endif

#if defined(I2C_USES_DMA) or defined(I2C_USES_IT) or defined(I2C_USES_WAIT)

		HAL_StatusTypeDef MsgReceivedCB(I2C_HandleTypeDef *hi2c);
#endif

//		template<typename T>
//		HAL_StatusTypeDef AttachCommInt(T *hint);
		template<typename T>
		HAL_StatusTypeDef AttachCommInt(T *hint, DMA_HandleTypeDef *hdmaRx = NULL, DMA_HandleTypeDef *hdmaTx = NULL);

	private:

		template<typename T>
		HAL_StatusTypeDef _PushObjToVect(T hint);
//		HAL_StatusTypeDef _PushObjToVect(CommInterface<I2C_HandleTypeDef>* hint);
//		HAL_StatusTypeDef _PushObjToVect(CommInterface<SPI_HandleTypeDef>* hint);
//		HAL_StatusTypeDef _PushObjToVect(CommInterface<UART_HandleTypeDef>* hint);

		template<typename T>
		CommInterface<T>* _GetObj(T *hint);

//		CommInterface<I2C_HandleTypeDef>* 	_GetObj(I2C_HandleTypeDef *hint);
//		CommInterface<SPI_HandleTypeDef>* 	_GetObj(SPI_HandleTypeDef *hint);
//		CommInterface<UART_HandleTypeDef>* 	_GetObj(UART_HandleTypeDef *hint);

		template<typename Handle, typename QueueVectTD>
		HAL_StatusTypeDef __MsgReceivedCB(Handle *IntHandle, QueueVectTD *Queue);
		uint8_t __CheckIfCommIntIsAttachedAndHasFreeSpace(CommIntUnionTypeDef *uCommInt, CommIntTypeDef eCommIntType);
		HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef *MsgInfo, uint8_t VectorIndex);
		HAL_StatusTypeDef __CheckAndSetCSPins(MessageInfoTypeDef *MsgInfo, uint8_t VectorIndex);
		template<typename queue>
		HAL_StatusTypeDef __CheckAndSetCSPinsGeneric(queue *VectQueue, uint8_t VectorIndex, MessageInfoTypeDef *MsgInfo);
		template<typename Handle, typename QueueVectTD>
		HAL_StatusTypeDef __CheckForNextCommRequestAndStart(Handle *IntHandle, QueueVectTD *Queue);


		std::vector<CommInterface<I2C_HandleTypeDef>*> _comm_I2C_vect;
		std::vector<CommInterface<SPI_HandleTypeDef>*> _comm_SPI_vect;
		std::vector<CommInterface<UART_HandleTypeDef>*> _comm_UART_vect;

// ToDo should be moved to objects
#if defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)

		std::vector<CommQueue<UART_HandleTypeDef*>> __huartQueueVect;
#endif

#if defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT)

		std::vector<CommQueue<SPI_HandleTypeDef*>> __hspiQueueVect;
#endif

#if defined(I2C_USES_DMA) or defined(I2C_USES_IT) or defined(I2C_USES_WAIT)

		std::vector<CommQueue<I2C_HandleTypeDef*>> __hi2cQueueVect;
#endif

#if defined(SPI_USES_DMA) or defined(I2C_USES_DMA) or defined(UART_USES_DMA)

		std::vector<DMA_HandleTypeDef*> __hdmaVect;
#endif

};
#endif

#include "CommManagerTemplateImpl.hpp"


#endif /* SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_ */
