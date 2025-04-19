/*
 * SpecialTypes.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */

#ifndef SPECIALTYPES_HPP_
#define SPECIALTYPES_HPP_
#include <functional>

//typedef struct MessageInfoTypeDef MessageInfoTypeDef;

typedef enum {
	COMM_INT_TXRX = 0,
	COMM_INT_RX,
	COMM_INT_TX,
	COMM_INT_MEM_TX,
	COMM_INT_MEM_RX
}CommIntTypeDef;

typedef enum {
	COMM_DMA = 0,
	COMM_INTERRUPT,
	COMM_WAIT,
	COMM_DUMMY
}CommModeTypeDef;

typedef union {
	UART_HandleTypeDef *huart;
	SPI_HandleTypeDef *hspi;
	I2C_HandleTypeDef *hi2c;
}CommIntUnionTypeDef;

template<typename T>
struct MessageInfoTypeDef{
	uint16_t context;
	CommIntTypeDef eCommType;
	T *IntHandle;
	uint16_t GPIO_PIN;
	GPIO_TypeDef *GPIOx;
	uint16_t len;
	uint8_t *pRxData;
	uint8_t *pTxData;
	uint16_t I2C_Addr;
	uint16_t I2C_MemAddr;
	uint8_t spi_cpol_high;
	std::function<void(MessageInfoTypeDef* MsgInfo)> *pCB;
	void (*pRxCompletedCB)(MessageInfoTypeDef* MsgInfo);
	void (*pTxCompletedCB)(MessageInfoTypeDef* MsgInfo);
	HAL_StatusTypeDef* TransactionStatus;
};



#endif /* SPECIALTYPES_HPP_ */
