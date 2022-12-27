/*
 * CommI2C.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */

#ifndef COMMMANAGER_COMMUART_HPP_
#define COMMMANAGER_COMMUART_HPP_

#include "CommInterface.hpp"

class CommUART : public CommInterface<UART_HandleTypeDef>
{
public:

	HAL_StatusTypeDef AttachCommInt(UART_HandleTypeDef *pIntStruct){return HAL_ERROR;};
	HAL_StatusTypeDef AttachCommInt(UART_HandleTypeDef *pIntStruct, DMA_HandleTypeDef *hdma){return HAL_ERROR;};

	HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef *MsgInfo) {return HAL_ERROR;};
	void CheckForNextCommRequestAndStart(void) {};
	HAL_StatusTypeDef CheckIfSameInstance(const UART_HandleTypeDef *pIntStruct){return HAL_ERROR;};
	const UART_HandleTypeDef* GetInstance(void) {return NULL;};

private:

	uint8_t CheckIfCommIntIsAttachedAndHasFreeSpace(CommIntTypeDef eCommIntType){return 0;};

};



#endif /* COMMMANAGER_COMMUART_HPP_ */
