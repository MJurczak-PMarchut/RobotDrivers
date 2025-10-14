/*
 * CommI2C.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */

#ifndef COMMMANAGER_COMMSPI_HPP_
#define COMMMANAGER_COMMSPI_HPP_

#include <CommBaseClass.hpp>

class CommSPI : public CommBaseClass<SPI_HandleTypeDef>
{
public:
	CommSPI(SPI_HandleTypeDef *hint, DMA_HandleTypeDef *hdma, CommModeTypeDef CommMode);
	HAL_StatusTypeDef MsgReceivedCB(SPI_HandleTypeDef *hint);

protected:
	virtual HAL_StatusTypeDef __CheckIfInterfaceFree(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo);
	virtual HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo);
private:
	uint8_t cpol_high = 0;
};



#endif /* COMMMANAGER_COMMSPI_HPP_ */
