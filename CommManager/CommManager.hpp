/*
 * CommManager.hpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#ifndef SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_
#define SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_
#include "../Configuration.h"

#ifdef MCU_STM32F4
#include "stm32f4xx_hal.h"
#elif MCU_STM32H7
#include "stm32h7xx_hal.h"
#endif

#include "vector"
#include "queue"

//TODO that is the dumbest way to do it possible, I don't like it, I should probably fix this in the future
typedef union {
	UART_HandleTypeDef *huart;
	SPI_HandleTypeDef *hspi;
	I2C_HandleTypeDef *hi2c;
}CommIntUnionTypeDef;


typedef enum {
	COMM_INT_SPI_TXRX, COMM_INT_SPI_RX, COMM_INT_UART_TX, COMM_INT_UART_RX, COMM_INT_I2C_TX, COMM_INT_I2C_RX
}CommIntTypeDef;

typedef struct MessageInfoTypeDef MessageInfoTypeDef;

struct MessageInfoTypeDef{
	CommIntUnionTypeDef uCommInt;
	CommIntTypeDef eCommType;
	uint16_t GPIO_PIN;
	GPIO_TypeDef *GPIOx;
	uint8_t len;
	uint8_t *pRxData;
	uint8_t *pTxData;
	uint8_t context;
	uint16_t I2C_Addr;
	void (*pRxCompletedCB)(struct MessageInfoTypeDef* MsgInfo);
	void (*pTxCompletedCB)(struct MessageInfoTypeDef* MsgInfo);
};

template <typename T>
struct CommQueue{
	T handle;
#if defined(SPI_USES_DMA) or defined(I2C_USES_DMA) or defined(UART_USES_DMA)
	DMA_HandleTypeDef *hdma;
#endif
	std::queue<MessageInfoTypeDef> MsgInfo;
	std::queue<MessageInfoTypeDef> MsgRx;
};

class CommManager
{
	public:
		CommManager();
		HAL_StatusTypeDef PushCommRequestIntoQueue(MessageInfoTypeDef *MsgInfo);
		HAL_StatusTypeDef CheckForNextCommRequestAndStart(I2C_HandleTypeDef *hi2c);
		HAL_StatusTypeDef MsgReceivedCB(UART_HandleTypeDef *huart);
		HAL_StatusTypeDef MsgReceivedCB(SPI_HandleTypeDef *hspi);
		HAL_StatusTypeDef MsgReceivedCB(I2C_HandleTypeDef *hi2c);
#ifdef UART_USES_DMA //Force user to provide DMA handle
		HAL_StatusTypeDef AttachCommInt(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);
#else
		HAL_StatusTypeDef AttachCommInt(UART_HandleTypeDef *huart);
#endif
#ifdef SPI_USES_DMA
		HAL_StatusTypeDef AttachCommInt(SPI_HandleTypeDef *hspi, DMA_HandleTypeDef *hdma);
#else
		HAL_StatusTypeDef AttachCommInt(SPI_HandleTypeDef *hspi);
#endif
#ifdef I2C_USES_DMA
		HAL_StatusTypeDef AttachCommInt(I2C_HandleTypeDef *hi2c, DMA_HandleTypeDef *hdma);
#else
		HAL_StatusTypeDef AttachCommInt(I2C_HandleTypeDef *hi2c);
#endif
	private:
		uint8_t __CheckIfCommIntIsAttachedAndHasFreeSpace(CommIntUnionTypeDef *uCommInt, CommIntTypeDef eCommIntType);
		std::vector<CommQueue<UART_HandleTypeDef*>> __huartQueueVect;
		std::vector<CommQueue<SPI_HandleTypeDef*>> __hspiQueueVect;
		std::vector<CommQueue<I2C_HandleTypeDef*>> __hi2cQueueVect;
		HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef *MsgInfo, uint8_t VectorIndex);
#if defined(SPI_USES_DMA) or defined(I2C_USES_DMA) or defined(UART_USES_DMA)
		std::vector<DMA_HandleTypeDef*> __hdmaVect;
#endif

};




#endif /* SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_ */
