/*
 * CommI2C.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */

#ifndef COMMMANAGER_COMMI2C_HPP_
#define COMMMANAGER_COMMI2C_HPP_

#include <CommBaseClass.hpp>

class CommI2C : public CommBaseClass<I2C_HandleTypeDef>
{
public:
	CommI2C(I2C_HandleTypeDef *hint, DMA_HandleTypeDef *hdma, CommModeTypeDef CommMode);

protected:
	virtual HAL_StatusTypeDef __CheckIfInterfaceFree(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);
	virtual HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);


};



#endif /* COMMMANAGER_COMMI2C_HPP_ */
