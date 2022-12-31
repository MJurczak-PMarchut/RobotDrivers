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

protected:
	virtual HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo);


};



#endif /* COMMMANAGER_COMMSPI_HPP_ */
