/*
 * CommI2C.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */

#ifndef COMMMANAGER_COMMUART_HPP_
#define COMMMANAGER_COMMUART_HPP_

#include <CommBaseClass.hpp>
#include "queue"

class CommUART : public CommBaseClass<UART_HandleTypeDef>
{
public:

	HAL_StatusTypeDef AttachCommInt(UART_HandleTypeDef *pIntStruct, CommModeTypeDef commType = COMM_DUMMY){return HAL_ERROR;};
	HAL_StatusTypeDef AttachCommInt(UART_HandleTypeDef *pIntStruct, DMA_HandleTypeDef *hdma, CommModeTypeDef commType = COMM_DUMMY){return HAL_ERROR;};

	HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo) {return HAL_ERROR;};
	void CheckForNextCommRequestAndStart(void) {};
	HAL_StatusTypeDef CheckIfSameInstance(const UART_HandleTypeDef *pIntStruct){return HAL_ERROR;};
	const UART_HandleTypeDef* GetInstance(void) {return NULL;};

	std::vector<CommQueue<UART_HandleTypeDef*>> __huartQueueVect;

private:

	uint8_t CheckIfCommIntIsAttachedAndHasFreeSpace(CommIntTypeDef eCommIntType){return 0;};

};



#endif /* COMMMANAGER_COMMUART_HPP_ */
