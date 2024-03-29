/*
 * ServoControl.cpp
 *
 *  Created on: Nov 17, 2021
 *      Author: Mateusz
 */
#include "../../RobotDrivers/ServoControl/ServoControl.hpp"

ServoControl::ServoControl(TIM_HandleTypeDef *htim)
{
	this->__htim = htim;
	this->__NoOfServosAttached = 0;
	this->__CurrentServoSched = 0;
#ifdef SERVO_CONTROL_INIT_TIM
	this->__InitTim(void);
#endif
}

HAL_StatusTypeDef ServoControl::__InitTim(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	this->__htim->Init.Prescaler = 41;
	this->__htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	this->__htim->Init.Period = 2000 - 1;
	this->__htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	this->__htim->Init.RepetitionCounter = 0;
	this->__htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(this->__htim) != HAL_OK)
	{
		return HAL_ERROR;
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(this->__htim, &sMasterConfig) != HAL_OK)
	{
		return HAL_ERROR;
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(this->__htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		return HAL_ERROR;
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(this->__htim, &sBreakDeadTimeConfig) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}


uint8_t ServoControl::AttachServo(GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN)
{
	ServoControlVectTypeDef Srv_control;

	if(this->__NoOfServosAttached < 8) //Max number of servos per channel
	{
		Srv_control.GPIO_PIN = GPIO_PIN;
		Srv_control.GPIOx = GPIOx;
		Srv_control.ServoAngle = 0;
		this->__NoOfServosAttached++;
		this->__ServoControlVect1.push_back(Srv_control);
		return this->__ServoControlVect1.size() - 1;
	}
	else if (this->__NoOfServosAttached < 16)
	{
		Srv_control.GPIO_PIN = GPIO_PIN;
		Srv_control.GPIOx = GPIOx;
		Srv_control.ServoAngle = 0;
		this->__ServoControlVect2.push_back(Srv_control);
		this->__NoOfServosAttached++;
		return this->__ServoControlVect1.size() + this->__ServoControlVect2.size() - 1;
	}
	else if (this->__NoOfServosAttached < 24)
	{
		Srv_control.GPIO_PIN = GPIO_PIN;
		Srv_control.GPIOx = GPIOx;
		Srv_control.ServoAngle = 0;
		this->__ServoControlVect3.push_back(Srv_control);
		this->__NoOfServosAttached++;
		return this->__ServoControlVect1.size() + this->__ServoControlVect2.size() + this->__ServoControlVect3.size() - 1 ;
	}
	else
	{
		return UNABLE_TO_ATTACH_SERVO;
	}

}

HAL_StatusTypeDef ServoControl::SetServoValue(uint8_t ServoNo, uint16_t ServoAngle)
{
	if((ServoAngle < 2000) && (ServoNo < __NoOfServosAttached))
	{
		if(ServoNo < 8)
		{
			this->__ServoControlVect1[ServoNo].ServoAngle = ServoAngle;
		}
		else if(ServoNo < 16)
		{
			this->__ServoControlVect2[ServoNo-8].ServoAngle = ServoAngle;
		}
		else if(ServoNo < 24)
		{
			this->__ServoControlVect3[ServoNo-16].ServoAngle = ServoAngle;
		}
		else
		{
			return HAL_ERROR;
		}
		return HAL_OK;
	}
	return HAL_ERROR;
}


HAL_StatusTypeDef ServoControl::StartServos(void)
{
	HAL_StatusTypeDef ret;
	//Set First PWM Values
	HAL_TIM_Base_Start_IT(this->__htim);
	if(this->__NoOfServosAttached > 0)
	{
		__HAL_TIM_SET_COMPARE(this->__htim, TIM_CHANNEL_1, this->__ServoControlVect1[0].ServoAngle + 510);
		//Start PWM
		ret = HAL_TIM_PWM_Start_IT(this->__htim, TIM_CHANNEL_1);
		if(ret == HAL_ERROR)
		{
			return ret;
		}
	}
	if(this->__NoOfServosAttached >= 8)
	{
		__HAL_TIM_SET_COMPARE(this->__htim, TIM_CHANNEL_2, this->__ServoControlVect2[0].ServoAngle + 510);
		//Start PWM
		ret = HAL_TIM_PWM_Start_IT(this->__htim, TIM_CHANNEL_2);
		if(ret == HAL_ERROR)
		{
			return ret;
		}
	}
	if(this->__NoOfServosAttached >= 16)
	{
		__HAL_TIM_SET_COMPARE(this->__htim, TIM_CHANNEL_3, this->__ServoControlVect3[0].ServoAngle + 510);
		//Start PWM
		ret = HAL_TIM_PWM_Start_IT(this->__htim, TIM_CHANNEL_3);
		if(ret == HAL_ERROR)
		{
			return ret;
		}
	}
	return HAL_OK;
}

uint8_t ServoControl::GetNoOfAttchedServos(void)
{
	return this->__NoOfServosAttached;
}

HAL_StatusTypeDef ServoControl::ServoControlCBHalfPulse(void)
{
	//Check what caused the Interrupt
	uint8_t Sched;
	Sched = (this->__CurrentServoSched >= 7)? 0: this->__CurrentServoSched + 1;
	if(this->__htim->Channel & HAL_TIM_ACTIVE_CHANNEL_1){
		if(this->__CurrentServoSched < this->__ServoControlVect1.size())
		{
			HAL_GPIO_WritePin(this->__ServoControlVect1[this->__CurrentServoSched].GPIOx, this->__ServoControlVect1[this->__CurrentServoSched].GPIO_PIN, GPIO_PIN_RESET);
		}
		if(Sched < this->__ServoControlVect1.size())
		{
			__HAL_TIM_SET_COMPARE(this->__htim, TIM_CHANNEL_1, this->__ServoControlVect1[Sched].ServoAngle + 500);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(this->__htim, TIM_CHANNEL_1, 500);
		}
	}
	if(this->__htim->Channel & HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if(this->__CurrentServoSched < this->__ServoControlVect2.size())
		{
			HAL_GPIO_WritePin(this->__ServoControlVect2[this->__CurrentServoSched].GPIOx, this->__ServoControlVect2[this->__CurrentServoSched].GPIO_PIN, GPIO_PIN_RESET);
		}
		if(Sched < this->__ServoControlVect2.size())
		{
			__HAL_TIM_SET_COMPARE(this->__htim, TIM_CHANNEL_2, this->__ServoControlVect2[Sched].ServoAngle + 500);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(this->__htim, TIM_CHANNEL_2, 500);
		}
	}
	if(this->__htim->Channel & HAL_TIM_ACTIVE_CHANNEL_3)
	{
		if(this->__CurrentServoSched < this->__ServoControlVect3.size())
		{
			HAL_GPIO_WritePin(this->__ServoControlVect3[this->__CurrentServoSched].GPIOx, this->__ServoControlVect3[this->__CurrentServoSched].GPIO_PIN, GPIO_PIN_RESET);
		}
		if(Sched < this->__ServoControlVect3.size())
		{
			__HAL_TIM_SET_COMPARE(this->__htim, TIM_CHANNEL_3, this->__ServoControlVect3[Sched].ServoAngle + 500);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(this->__htim, TIM_CHANNEL_3, 500);
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef ServoControl::ServoControlCBUpdate(void)
{
	this->__CurrentServoSched = (this->__CurrentServoSched < 7)? this->__CurrentServoSched + 1 : 0;
	if(this->__CurrentServoSched < this->__ServoControlVect1.size())
	{
		HAL_GPIO_WritePin(this->__ServoControlVect1[this->__CurrentServoSched].GPIOx, this->__ServoControlVect1[this->__CurrentServoSched].GPIO_PIN, GPIO_PIN_SET);
	}
	if(this->__CurrentServoSched < this->__ServoControlVect2.size())
	{
		HAL_GPIO_WritePin(this->__ServoControlVect2[this->__CurrentServoSched].GPIOx, this->__ServoControlVect2[this->__CurrentServoSched].GPIO_PIN, GPIO_PIN_SET);
	}
	if(this->__CurrentServoSched < this->__ServoControlVect3.size())
	{
		HAL_GPIO_WritePin(this->__ServoControlVect3[this->__CurrentServoSched].GPIOx, this->__ServoControlVect3[this->__CurrentServoSched].GPIO_PIN, GPIO_PIN_SET);
	}
	return HAL_OK;
}
