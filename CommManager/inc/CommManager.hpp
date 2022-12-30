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

class CommManager
{
	public:
		CommManager();

//		template<typename T>
		HAL_StatusTypeDef PushCommRequestIntoQueue(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);
		HAL_StatusTypeDef PushCommRequestIntoQueue(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo);
		HAL_StatusTypeDef PushCommRequestIntoQueue(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo);

		HAL_StatusTypeDef MsgReceivedCB(UART_HandleTypeDef *huart, uint16_t len);
		HAL_StatusTypeDef MsgReceivedCB(UART_HandleTypeDef *huart);
		HAL_StatusTypeDef MsgReceivedCB(SPI_HandleTypeDef *hspi);
		HAL_StatusTypeDef MsgReceivedCB(I2C_HandleTypeDef *hi2c);

		template<typename T>
		HAL_StatusTypeDef AttachCommInt(T *hint, DMA_HandleTypeDef *hdmaRx = NULL, DMA_HandleTypeDef *hdmaTx = NULL);

	private:
		template<typename T>
		HAL_StatusTypeDef _PushObjToVect(T hint);

		template<typename queue, typename T>
		CommBaseClass<T>* _MatchInstance(queue *VectQueue, T *hint);

		template<typename T>
		CommBaseClass<T>* _GetObj(T *hint);

		template<typename T>
		HAL_StatusTypeDef __CheckAndSetCSPins(MessageInfoTypeDef<T> *MsgInfo, uint8_t VectorIndex);

		template<typename queue, typename T>
		HAL_StatusTypeDef __CheckAndSetCSPinsGeneric(queue *VectQueue, uint8_t VectorIndex, MessageInfoTypeDef<T> *MsgInfo);


		std::vector<CommBaseClass<I2C_HandleTypeDef>*> _comm_I2C_vect;
		std::vector<CommBaseClass<SPI_HandleTypeDef>*> _comm_SPI_vect;
		std::vector<CommBaseClass<UART_HandleTypeDef>*> _comm_UART_vect;


};
#endif

#include "CommManagerTemplateImpl.hpp"


#endif /* SUMODRIVERS_COMMMANAGER_COMMMANAGER_HPP_ */
