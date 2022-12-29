/*
 * CommI2C.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */

#ifndef COMMMANAGER_COMMSPI_HPP_
#define COMMMANAGER_COMMSPI_HPP_

#include "CommInterface.hpp"

class CommSPI : public CommInterface<SPI_HandleTypeDef>
{
public:

	HAL_StatusTypeDef AttachCommInt(SPI_HandleTypeDef *pIntStruct){return HAL_ERROR;};
	HAL_StatusTypeDef AttachCommInt(SPI_HandleTypeDef *pIntStruct, DMA_HandleTypeDef *hdma){return HAL_ERROR;};

	HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo) {return HAL_ERROR;};
	void CheckForNextCommRequestAndStart(void) {};
	HAL_StatusTypeDef CheckIfSameInstance(const SPI_HandleTypeDef *pIntStruct){return HAL_ERROR;};
	const SPI_HandleTypeDef* GetInstance(void) {return NULL;};

private:

	uint8_t CheckIfCommIntIsAttachedAndHasFreeSpace(CommIntTypeDef eCommIntType){return 0;};

};



#endif /* COMMMANAGER_COMMSPI_HPP_ */
