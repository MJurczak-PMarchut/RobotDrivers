/*
 * VESC.hpp
 *
 *  Created on: 26 mar 2023
 *      Author: Mateusz
 */

#ifndef VESC_VESC_HPP_
#define VESC_VESC_HPP_
#include "MotorControl.hpp"
#include "RobotSpecificDefines.hpp"
#include "VescUart.h"

#ifdef USES_RTOS
#include "osapi.h"
#endif

class VESC : protected MCInterface
{
public:
	VESC(MotorSideTypeDef side, UART_HandleTypeDef *huart, CommManager *CommunicationManager);
	~VESC(){};
	HAL_StatusTypeDef Init();
	HAL_StatusTypeDef SetMotorPowerPWM(uint16_t PowerPWM);
	HAL_StatusTypeDef SetMotorDirection(MotorDirectionTypeDef Dir);
	HAL_StatusTypeDef Disable(void);
	HAL_StatusTypeDef Enable(void);
	HAL_StatusTypeDef EmergencyStop(void);
	HAL_StatusTypeDef CheckControllerState(void);
	HAL_StatusTypeDef CheckIfControllerInitializedOk(void);

private:
	HAL_StatusTypeDef SetMotorCurrent(float current);
	static VescStream *_VescStream;
	static VescUart *_Vesc;
	MotorSideTypeDef _side;
	int16_t CurrentDutyCycle;
	MotorDirectionTypeDef _Dir;
	uint8_t enable_status;
};



#endif /* VESC_VESC_HPP_ */
