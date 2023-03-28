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
#include <CommBaseClass.hpp>

#define SPI SPI_HandleTypeDef
#define I2C I2C_HandleTypeDef
#define UART UART_HandleTypeDef

class CommManager
{
	public:
		CommManager();

		template<typename T>
		HAL_StatusTypeDef PushCommRequestIntoQueue(MessageInfoTypeDef<T> *MsgInfo);

		HAL_StatusTypeDef MsgReceivedCB(UART_HandleTypeDef *huart, uint16_t len);
		HAL_StatusTypeDef MsgReceivedRxCB(UART_HandleTypeDef *hint);

		template<typename T>
		HAL_StatusTypeDef MsgReceivedCB(T *hint);

		template<typename T>
		HAL_StatusTypeDef AttachCommInt(T *hint, CommModeTypeDef CommMode, DMA_HandleTypeDef *hdmaRx = NULL, DMA_HandleTypeDef *hdmaTx = NULL);

	private:

		template<typename queue, typename T>
		CommBaseClass<T>* _MatchInstance(queue *VectQueue, T *hint);

		template<typename T>
		std::vector<CommBaseClass<T>*>* _GetVector(T *hint);

		template<typename T>
		CommBaseClass<T>* _GetObj(T *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx, CommModeTypeDef CommMode);

		std::vector<CommBaseClass<I2C_HandleTypeDef>*> _comm_I2C_vect;
		std::vector<CommBaseClass<SPI_HandleTypeDef>*> _comm_SPI_vect;
		std::vector<CommBaseClass<UART_HandleTypeDef>*> _comm_UART_vect;


};
#endif
#include "CommManagerTemplateImpl.hpp"

#endif /* SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_ */
