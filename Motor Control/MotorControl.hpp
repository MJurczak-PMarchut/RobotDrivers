/*
 * MotorControl.hpp
 *
 *  Created on: 12 lis 2021
 *      Author: Mateusz
 */
#include "stm32f4xx_hal.h"
#ifndef SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_
#define SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_

typedef enum {MOTOR_LEFT, MOTOR_RIGHT} MotorSide;

class MCInterface{
	public:
	virtual HAL_StatusTypeDef SetMotorPower(uint8_t power, MotorSide motor);
	virtual HAL_StatusTypeDef Set

};

#endif /* SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_ */
