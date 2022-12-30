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
	CommUART(UART_HandleTypeDef *hint, DMA_HandleTypeDef *hdmaRx, DMA_HandleTypeDef *hdmaTx);

protected:
	virtual HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo);


private:
	DMA_HandleTypeDef *_hdmaTx;

};



#endif /* COMMMANAGER_COMMUART_HPP_ */
