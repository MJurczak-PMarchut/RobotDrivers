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

//#include "L9960T.hpp"
//#include "VESC/VescUart.h"
//#include "MotorControl.hpp"

//TODO that is the dumbest way to do it possible, I don't like it, I should probably fix this in the future

typedef union {
#if defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)
	UART_HandleTypeDef *huart;
#endif
#if defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT)
	SPI_HandleTypeDef *hspi;
#endif
#if defined(I2C_USES_DMA) or defined(I2C_USES_IT) or defined(I2C_USES_WAIT)
	I2C_HandleTypeDef *hi2c;
#endif
}CommIntUnionTypeDef;
//
//typedef union {
//	L9960T *L9960T_Controller;
//	VescUart *VESC_Controller;
//}MotorControllerUnionTypeDef;
//
//typedef struct
//{
//	MotorControllerUnionTypeDef MCUnion;
//	ControllerTypeTypeDef ControllerType;
//}MotorControllerCallbackTD;

typedef enum {
	COMM_INT_SPI_TXRX, COMM_INT_SPI_RX, COMM_INT_UART_TX, COMM_INT_UART_RX, COMM_INT_I2C_TX, COMM_INT_I2C_RX
}CommIntTypeDef;

typedef struct MessageInfoTypeDef MessageInfoTypeDef;

struct MessageInfoTypeDef{
	CommIntUnionTypeDef uCommInt;
	CommIntTypeDef eCommType;
	uint16_t GPIO_PIN;
	GPIO_TypeDef *GPIOx;
	uint16_t len;
	uint8_t *pRxData;
	uint8_t *pTxData;
	uint16_t context;
	uint16_t I2C_Addr;
	void (*pRxCompletedCB)(struct MessageInfoTypeDef* MsgInfo);
	void (*pTxCompletedCB)(struct MessageInfoTypeDef* MsgInfo);
//	MotorControllerCallbackTD CommCompletedTB;
};

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

#ifdef UART_USES_DMA //Force user to provide DMA handle

		HAL_StatusTypeDef AttachCommInt(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx);

#elif defined(UART_USES_IT) or defined(UART_USES_WAIT)

		HAL_StatusTypeDef AttachCommInt(UART_HandleTypeDef *huart);
#endif

#ifdef SPI_USES_DMA

		HAL_StatusTypeDef AttachCommInt(SPI_HandleTypeDef *hspi, DMA_HandleTypeDef *hdma);

#elif defined(SPI_USES_IT) or defined(SPI_USES_WAIT)

		HAL_StatusTypeDef AttachCommInt(SPI_HandleTypeDef *hspi);
#endif

#ifdef I2C_USES_DMA

		HAL_StatusTypeDef AttachCommInt(I2C_HandleTypeDef *hi2c, DMA_HandleTypeDef *hdma);
#elif defined(I2C_USES_IT) or defined(I2C_USES_WAIT)

		HAL_StatusTypeDef AttachCommInt(I2C_HandleTypeDef *hi2c);
#endif

	private:
		template<typename Handle, typename QueueVectTD>
		HAL_StatusTypeDef __MsgReceivedCB(Handle *IntHandle, QueueVectTD *Queue);
		uint8_t __CheckIfCommIntIsAttachedAndHasFreeSpace(CommIntUnionTypeDef *uCommInt, CommIntTypeDef eCommIntType);
		HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef *MsgInfo, uint8_t VectorIndex);
		HAL_StatusTypeDef __CheckAndSetCSPins(MessageInfoTypeDef *MsgInfo, uint8_t VectorIndex);
		template<typename queue>
		HAL_StatusTypeDef __CheckAndSetCSPinsGeneric(queue *VectQueue, uint8_t VectorIndex, MessageInfoTypeDef *MsgInfo);
		template<typename Handle, typename QueueVectTD>
		HAL_StatusTypeDef __CheckForNextCommRequestAndStart(Handle *IntHandle, QueueVectTD *Queue);

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



#endif /* SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_ */
