/*
 * CommI2C.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */

#ifndef COMMMANAGER_COMMI2C_HPP_
#define COMMMANAGER_COMMI2C_HPP_

#include "CommInterface.hpp"

class CommI2C : public CommInterface<I2C_HandleTypeDef>
{
public:

	HAL_StatusTypeDef AttachCommInt(I2C_HandleTypeDef *pIntStruct){return HAL_ERROR;};
	HAL_StatusTypeDef AttachCommInt(I2C_HandleTypeDef *pIntStruct, DMA_HandleTypeDef *hdma){return HAL_ERROR;};

	HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo) {return HAL_ERROR;};
	void CheckForNextCommRequestAndStart(void) {};
	HAL_StatusTypeDef CheckIfSameInstance(const I2C_HandleTypeDef *pIntStruct){return HAL_ERROR;};
	const I2C_HandleTypeDef* GetInstance(void) {return NULL;};

private:

	uint8_t CheckIfCommIntIsAttachedAndHasFreeSpace(CommIntTypeDef eCommIntType){return 0;};

};



#endif /* COMMMANAGER_COMMI2C_HPP_ */
