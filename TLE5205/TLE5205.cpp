/*
 * TLE5205.cpp
 *
 *  Created on: Nov 28, 2021
 *      Author: Mateusz
 */

#include "../../RobotDrivers/TLE5205/TLE5205.hpp"

#define FREEWHELING 0b11
#define MOTOR_CCW 0b10
#define MOTOR_CW 0b00
#define BRAKE 0b01
//typedef  enum {
//	FREEWHELING = 0b11,
//	MOTOR_CCW = 0b10,
//	MOTOR_CW = 0b00,
//	BRAKE = 0b01
//}TLE5205_STATE_TypeDef;

TLE5205::TLE5205(MotorSideTypeDef side, TIM_HandleTypeDef *htim, uint32_t Channel): MCInterface(side)
{
	__htim = htim;
	__Dir = MOTOR_DIR_FORWARD;
	__PowerPWM = 0;
	__IN1_STATE = (GPIO_PinState)(FREEWHELING & 0x1);
	__IN2_STATE = (GPIO_PinState)(FREEWHELING & 0x2);
	__Channel = Channel;
	if(side == MOTOR_LEFT)
	{
		__IN1_PIN = MOTOR_LEFT_IN1_Pin;
		__IN2_PIN = MOTOR_LEFT_IN2_Pin;
		__IN1_PORT = MOTOR_LEFT_IN1_Port;
		__IN2_PORT = MOTOR_LEFT_IN2_Port;
	}
	else if(side == MOTOR_RIGHT)
	{
		__IN1_PIN = MOTOR_RIGHT_IN1_Pin;
		__IN2_PIN = MOTOR_RIGHT_IN2_Pin;
		__IN1_PORT = MOTOR_RIGHT_IN1_Port;
		__IN2_PORT = MOTOR_RIGHT_IN2_Port;
	}
	__Motor_State = MOTOR_DISABLED;
}


HAL_StatusTypeDef TLE5205::SetMotorPowerPWM(uint16_t PowerPWM)
{
	if(PowerPWM < 1000)
	{
		__PowerPWM = PowerPWM;
		__HAL_TIM_SET_COMPARE(this->__htim, this->__Channel, PowerPWM);
		if(PowerPWM == 0)
		{
			__IN1_STATE = (GPIO_PinState)(FREEWHELING & 0x1);
			__IN2_STATE = (GPIO_PinState)((FREEWHELING & 0x2) >> 1);
		}
		SetMotorDirection(__Dir);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef TLE5205::SetMotorDirection(MotorDirectionTypeDef Dir)
{
	__Dir = Dir;
	if(Dir == MOTOR_DIR_FORWARD)
	{
		__IN1_STATE = (GPIO_PinState)(MOTOR_CW & 0x1);
		__IN2_STATE = (GPIO_PinState)((MOTOR_CW & 0x2) >> 1);
	}
	else
	{
		__IN1_STATE = (GPIO_PinState)(MOTOR_CCW & 0x1);
		__IN2_STATE = (GPIO_PinState)((MOTOR_CCW & 0x2) >> 1);
	}
	return HAL_OK;
}

HAL_StatusTypeDef TLE5205::Disable(void)
{
	__PowerPWM = 0;
	__HAL_TIM_SET_COMPARE(this->__htim, this->__Channel, 0);
	__Motor_State = MOTOR_DISABLED;
	__IN1_STATE = (GPIO_PinState)(FREEWHELING & 0x1);
	__IN2_STATE = (GPIO_PinState)((FREEWHELING & 0x2) >> 1);
	return HAL_OK;
}

HAL_StatusTypeDef TLE5205::Brake(void)
{

	__IN1_STATE = (GPIO_PinState)(BRAKE & 0x1);
	__IN2_STATE = (GPIO_PinState)(BRAKE & 0x2);
	return HAL_OK;
}

HAL_StatusTypeDef TLE5205::Enable(void)
{
	__Motor_State = MOTOR_ENABLED;
	SetMotorDirection(__Dir);
	return HAL_OK;
}

HAL_StatusTypeDef TLE5205::EmergencyStop(void)
{
	return this->Disable();
}

void TLE5205::TimCB(void)
{
//		HAL_GPIO_WritePin(__IN1_PORT, __IN1_PIN, __IN1_STATE);
//		HAL_GPIO_WritePin(__IN2_PORT, __IN2_PIN, __IN2_STATE);
		__IN1_PORT->BSRR = __IN1_PIN << ((__IN1_STATE)?GPIO_NUMBER_SUMO:0);
		__IN2_PORT->BSRR = __IN2_PIN << ((__IN2_STATE)?GPIO_NUMBER_SUMO:0);
}

void TLE5205::TimCBPulse(void)
{
#if (FREEWHELING & 0x1)
	__IN1_PORT->BSRR = __IN1_PIN << GPIO_NUMBER_SUMO;
#else
	__IN1_PORT->BSRR = __IN1_PIN;
#endif
#if (FREEWHELING & 0x2)
	__IN1_PORT->BSRR = __IN1_PIN << GPIO_NUMBER_SUMO;

#else
	__IN1_PORT->BSRR = __IN1_PIN;
#endif

//	HAL_GPIO_WritePin(__IN1_PORT, __IN1_PIN, (GPIO_PinState)(FREEWHELING & 0x1));
//	HAL_GPIO_WritePin(__IN2_PORT, __IN2_PIN, (GPIO_PinState)((FREEWHELING & 0x2) >> 1));
}
