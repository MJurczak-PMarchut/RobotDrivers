/*
 * VESC.cpp
 *
 *  Created on: 26 mar 2023
 *      Author: Mateusz
 */


#include "VESC.hpp"
#include "string.h"
VescStream *VESC::_VescStream = 0;;
VescUart *VESC::_Vesc = 0;

VESC* VESC::ptrVESC[2] ={0};

VESC::VESC(MotorSideTypeDef side, UART_HandleTypeDef *huart, CommManager *CommunicationManager) :
		_side(side),
		_CanID((side == MOTOR_LEFT)?VESC_LEFT_CANID : VESC_RIGHT_CANID),
		_DirectionInverted(false)
{
	if(_VescStream == NULL){
		_VescStream = new VescStream(CommunicationManager, huart);
		_Vesc = new VescUart(_VescStream);
	}
#ifdef LEFT_MOTOR_INVERT_DIRECTION
	if(_side == MOTOR_LEFT){
		DirectionInverted = true;
	}
#endif

#ifdef RIGHT_MOTOR_INVERT_DIRECTION
	if(_side == MOTOR_RIGHT){
		DirectionInverted = true;
	}
#endif
	ptrVESC[NoOfControllers-1] = this;
	_VescData = &_Vesc->data;

}

void VESC::SetVESCData(void)
{
	for(uint8_t i = 0; i < NoOfControllers; i++)
	{
		if(ptrVESC[i]->_CanID == _Vesc->data.id)
		{
			memcpy((void*)&ptrVESC[i]->Data, (void*)&_Vesc->data, sizeof(_Vesc->data));
			ptrVESC[i]->Data.rpm = ptrVESC[i]->Data.rpm / MOTOR_POLE_PAIRS;
		}
	}
}

HAL_StatusTypeDef VESC::Init()
{
	_VescStream->Start();
	Disable();
	SetMotorDirection(MOTOR_DIR_FORWARD);
	return HAL_OK;
}

HAL_StatusTypeDef VESC::SetMotorPowerPWM(uint16_t PowerPWM)
{
	CurrentDutyCycle = ((_Dir == MOTOR_DIR_FORWARD) != _DirectionInverted)? PowerPWM:-PowerPWM;
	if(enable_status){
		_Vesc->setDuty((int32_t)CurrentDutyCycle, _CanID);
	}
	return HAL_OK;
}

HAL_StatusTypeDef VESC::SetMotorDirection(MotorDirectionTypeDef Dir)
{
	_Dir = Dir;
	if(enable_status){
		_Vesc->setDuty((int32_t)CurrentDutyCycle, _CanID);
	}
	return HAL_OK;
}

HAL_StatusTypeDef VESC::Disable(void)
{
	enable_status = 0;
	_Vesc->setDuty((int32_t)0, _CanID);
	CurrentDutyCycle = 0;
	return HAL_OK;
}

HAL_StatusTypeDef VESC::Enable(void)
{
	enable_status = 10;
	return HAL_OK;
}

HAL_StatusTypeDef VESC::EmergencyStop(void)
{
	return Disable();
}

HAL_StatusTypeDef VESC::CheckControllerState(void)
{
	_Vesc->sendKeepalive(_CanID);
	_Vesc->getVescValues(_CanID);
	SetVESCData();
	return HAL_OK;
}

HAL_StatusTypeDef VESC::CheckIfControllerInitializedOk(void)
{
	return HAL_OK;
}

HAL_StatusTypeDef VESC::SetMotorCurrent(float current)
{
	_Vesc->setCurrent(current, _CanID);
	return HAL_OK;
}
