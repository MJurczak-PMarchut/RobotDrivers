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

	HAL_StatusTypeDef AttachCommInt(SPI_HandleTypeDef *pIntStruct, CommModeTypeDef commType = COMM_DUMMY){return HAL_ERROR;};
	HAL_StatusTypeDef AttachCommInt(SPI_HandleTypeDef *pIntStruct, DMA_HandleTypeDef *hdma, CommModeTypeDef commType = COMM_DUMMY){return HAL_ERROR;};

	HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo) {return HAL_ERROR;};
	void CheckForNextCommRequestAndStart(void) {};
	HAL_StatusTypeDef CheckIfSameInstance(const SPI_HandleTypeDef *pIntStruct){return HAL_ERROR;};
	const SPI_HandleTypeDef* GetInstance(void) {return NULL;};

protected:
	virtual HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo);

private:

	uint8_t CheckIfCommIntIsAttachedAndHasFreeSpace(CommIntTypeDef eCommIntType){return 0;};

};



#endif /* COMMMANAGER_COMMSPI_HPP_ */
