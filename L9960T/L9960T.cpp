/*
 * L9960T.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#include "L9960T.hpp"


L9960T::L9960T(MotorSideTypeDef side, SPI_HandleTypeDef *hspi,  uint16_t CS_Pin, GPIO_TypeDef CS_Port, CommManager *CommunicationManager)
{

	this->__CommunicationManager = CommunicationManager;
	this->__hspi = hspi;
	this->__side = side;
	this->__CS_Pin = CS_Pin;
	this->__CS_Port = CS_Port;

	if((side == MOTOR_LEFT) && ((__Instantiated_sides & (1 << MOTOR_LEFT)) == 0))
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_A_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_A_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_A_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_A_GPIO_Port;
		this->__Instantiated_sides |= (1 << MOTOR_LEFT);
	}
	else if ((side == MOTOR_RIGHT) && ((__Instantiated_sides & (1 << MOTOR_RIGHT)) == 0))
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_B_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_B_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_B_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_B_GPIO_Port;
		this->__Instantiated_sides |= (1 << MOTOR_RIGHT);
	}
}

HAL_StatusTypeDef L9960T::AttachPWMTimerAndChannel(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	assert_param(IS_TIM_CHANNELS(Channel));
	this->__htim = htim;
	this->__Channel = Channel;
	return HAL_OK;
}

HAL_StatusTypeDef L9960T::SetMotorPowerPWM(uint16_t PowerPWM)
{
	if(PowerPWM < 1000)
	{
		__HAL_TIM_SET_COMPARE(this->__htim, this->__Channel, PowerPWM);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef L9960T::SetMotorDirection(MotorDirectionTypeDef Dir)
{

	//TODO Check in what mode we are, and change direction
	if(Dir == MOTOR_DIR_FORWARD)
	{

		return HAL_OK;
	}
	else if(Dir == MOTOR_DIR_BACKWARD)
	{

		return HAL_OK;
	}
	return HAL_ERROR;
}

