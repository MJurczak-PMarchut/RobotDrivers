/*
 * SpecialTypes.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */

#ifndef SPECIALTYPES_HPP_
#define SPECIALTYPES_HPP_
#include <functional>

typedef struct MessageInfoTypeDef MessageInfoTypeDef;

typedef enum {
	COMM_INT_SPI_TXRX = 0,
	COMM_INT_SPI_RX,

	COMM_INT_UART_TX,
	COMM_INT_UART_RX,

	COMM_INT_I2C_TX,
	COMM_INT_I2C_RX,

	COMM_INT_I2C_MEM_TX,
	COMM_INT_I2C_MEM_RX
}CommIntTypeDef;

typedef union {
	UART_HandleTypeDef *huart;
	SPI_HandleTypeDef *hspi;
	I2C_HandleTypeDef *hi2c;
}CommIntUnionTypeDef;

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
	uint16_t I2C_MemAddr;
	std::function<void(MessageInfoTypeDef* MsgInfo)> pCB;
	void (*pRxCompletedCB)(MessageInfoTypeDef* MsgInfo);
	void (*pTxCompletedCB)(MessageInfoTypeDef* MsgInfo);
	HAL_StatusTypeDef* TransactionStatus;
};


#endif /* SPECIALTYPES_HPP_ */
