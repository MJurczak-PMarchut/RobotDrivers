/*
 * VESC.cpp
 *
 *  Created on: 26 mar 2023
 *      Author: Mateusz
 */


#include "VESC.hpp"

VescStream *VESC::_VescStream = 0;;
VescUart *VESC::_Vesc = 0;

VESC::VESC(MotorSideTypeDef side, UART_HandleTypeDef *huart, CommManager *CommunicationManager) : _side(side)
{
	if(_VescStream == NULL){
		_VescStream = new VescStream(CommunicationManager, huart);
		_Vesc = new VescUart(_VescStream);
	}
}

HAL_StatusTypeDef VESC::Init()
{
	Disable();
	SetMotorCurrent(60);
	SetMotorDirection(MOTOR_DIR_FORWARD);
	return HAL_OK;
}

HAL_StatusTypeDef VESC::SetMotorPowerPWM(uint16_t PowerPWM)
{
	CurrentDutyCycle = (_Dir == MOTOR_DIR_FORWARD)? PowerPWM:-PowerPWM;
	if(enable_status){
		_Vesc->setDuty((int32_t)CurrentDutyCycle, (uint8_t)_side);
	}
	return HAL_OK;
}

HAL_StatusTypeDef VESC::SetMotorDirection(MotorDirectionTypeDef Dir)
{
	_Dir = Dir;
	if(enable_status){
		_Vesc->setDuty((int32_t)CurrentDutyCycle, (uint8_t)_side);
	}
	return HAL_OK;
}

HAL_StatusTypeDef VESC::Disable(void)
{
	enable_status = 0;
	_Vesc->setDuty((int32_t)0, (uint8_t)_side);
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
	_Vesc->sendKeepalive((uint8_t)_side);
	return HAL_OK;
}

HAL_StatusTypeDef VESC::CheckIfControllerInitializedOk(void)
{
	return HAL_OK;
}

HAL_StatusTypeDef VESC::SetMotorCurrent(float current)
{
	_Vesc->setCurrent(current, (uint8_t)_side);
	return HAL_OK;
}
