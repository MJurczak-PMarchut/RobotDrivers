/*
 * CommManager.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */
#include "CommManager.hpp"

#include "vector"

#ifndef NO_COMMUNICATION_INTERFACE_DEFINED
#ifndef MAX_MESSAGE_NO_IN_QUEUE
#define MAX_MESSAGE_NO_IN_QUEUE 5
#endif

#include "CommUART.hpp"
#include "CommSPI.hpp"
#include "CommI2C.hpp"

#define VECTOR_NOT_FOUND 0xFF

CommManager::CommManager()
{

}

HAL_StatusTypeDef CommManager::MsgReceivedCB(UART_HandleTypeDef *huart, uint16_t len)
{
	HAL_StatusTypeDef ret_value = HAL_ERROR;
	CommBaseClass<UART_HandleTypeDef>* handle_inst;
	handle_inst = this->_MatchInstance(&_comm_UART_vect, huart);
	if(handle_inst != NULL)
	{
		ret_value = handle_inst->MsgReceivedCB(huart, len);
	}
	return ret_value;
}

HAL_StatusTypeDef CommManager::MsgReceivedRxCB(UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef ret_value = HAL_ERROR;
	CommBaseClass<UART_HandleTypeDef>* handle_inst;
	handle_inst = this->_MatchInstance(&_comm_UART_vect, huart);
	if(handle_inst != NULL)
	{
		ret_value = handle_inst->MsgReceivedRxCB(huart);
	}
	return ret_value;
}

template<>
std::vector<CommBaseClass<I2C_HandleTypeDef>*>* CommManager::_GetVector(I2C_HandleTypeDef *hint)
{
	return &_comm_I2C_vect;
}

template<>
std::vector<CommBaseClass<SPI_HandleTypeDef>*>* CommManager::_GetVector(SPI_HandleTypeDef *hint)
{
	return &_comm_SPI_vect;
}

template<>
std::vector<CommBaseClass<UART_HandleTypeDef>*>* CommManager::_GetVector(UART_HandleTypeDef *hint)
{
	return &_comm_UART_vect;
}

template<>
CommBaseClass<UART_HandleTypeDef>* CommManager::_GetObj(UART_HandleTypeDef *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx, CommModeTypeDef CommMode)
{
	return new CommUART(hint, hdmaRx, hdmaTx, CommMode);
}

template<>
CommBaseClass<SPI_HandleTypeDef>* CommManager::_GetObj(SPI_HandleTypeDef *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx, CommModeTypeDef CommMode)
{
	return new CommSPI(hint, hdmaRx, CommMode);
}

template<>
CommBaseClass<I2C_HandleTypeDef>* CommManager::_GetObj(I2C_HandleTypeDef *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx, CommModeTypeDef CommMode)
{
	return new CommI2C(hint, hdmaRx, CommMode);
}

#endif
