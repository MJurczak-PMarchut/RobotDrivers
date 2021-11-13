/*
 * CommManager.hpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#ifndef SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_
#define SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_
#include "stm32f4xx_hal.h"

//TODO that is the dumbest way to do it possible, I don't like it, I should probably fix this in the future
typedef union {
	UART_HandleTypeDef *huart;
	SPI_HandleTypeDef *hspi;
	I2C_HandleTypeDef *hi2c;
}CommIntUnionTypeDef;

typedef enum {
	COMM_INT_SPI_TXRX, COMM_INT_SPI_RX, COMM_INT_UART_TXRX, COMM_INT_SPI_RX, COMM_INT_I2C_TXRX, COMM_INT_I2C_RX
}CommIntTypeDef;


typedef struct {
	CommIntUnionTypeDef uCommInt;
	CommIntTypeDef eCommType;
	uint16_t GPIO_PIN;
	GPIO_TypeDef *GPIOx;
	uint8_t lenRx;
	uint8_t lenTx;
	uint8_t *pRxData;
	uint8_t *pTxData;
	uint8_t context;
	void (*pRxCompletedCB)(struct MessageInfoTypeDef* MsgInfo);
	void (*pTxCompletedCB)(struct MessageInfoTypeDef* MsgInfo);
}MessageInfoTypeDef;

class CommManager
{
	public:
		CommManager();
		HAL_StatusTypeDef GetContext(MessageInfoTypeDef *MsgInfo);
		HAL_StatusTypeDef PushCommRequestIntoQueue(MessageInfoTypeDef MsgInfo);
		HAL_StatusTypeDef GetNextCommRequest(MessageInfoTypeDef MsgInfo);
};




#endif /* SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_ */
