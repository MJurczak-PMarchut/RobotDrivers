/*
 * CommI2C.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */

#ifndef COMMMANAGER_COMMUART_HPP_
#define COMMMANAGER_COMMUART_HPP_

#include <CommBaseClass.hpp>

class CommUART : public CommBaseClass<UART_HandleTypeDef>
{
public:
	CommUART(UART_HandleTypeDef *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx, CommModeTypeDef CommMode);
	HAL_StatusTypeDef MsgReceivedCB(UART_HandleTypeDef *hint, uint16_t len);
	HAL_StatusTypeDef MsgReceivedRxCB(UART_HandleTypeDef *hint);
	HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo);
protected:
	virtual HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo);
	HAL_StatusTypeDef __CheckForNextRxCommRequestAndStart();


private:
	DMA_HandleTypeDef *_hdmaTx;
	std::queue<MessageInfoTypeDef<UART_HandleTypeDef>> _RxMsgQueue;

};



#endif /* COMMMANAGER_COMMUART_HPP_ */
