/*
 * TLE5205.hpp
 *
 *  Created on: Nov 28, 2021
 *      Author: Mateusz
 */

#ifndef TLE5205_TLE5205_HPP_
#define TLE5205_TLE5205_HPP_
#include "Configuration.h"
#include "MotorControl.hpp"
#include "RobotSpecificDefines.hpp"



class TLE5205 : protected MCInterface{
	public:
		TLE5205(MotorSideTypeDef side, TIM_HandleTypeDef *htim, uint32_t Channel);
		HAL_StatusTypeDef SetMotorPowerPWM(uint16_t PowerPWM);
		HAL_StatusTypeDef SetMotorDirection(MotorDirectionTypeDef Dir);
		HAL_StatusTypeDef Disable(void);
		HAL_StatusTypeDef Enable(void);
		HAL_StatusTypeDef EmergencyStop(void);
		HAL_StatusTypeDef Brake(void);
		void TimCB(void);
		void TimCBPulse(void);
	private:
		uint16_t __IN1_PIN;
		uint16_t __IN2_PIN;
		GPIO_TypeDef *__IN1_PORT;
		GPIO_TypeDef *__IN2_PORT;
		GPIO_PinState __IN1_STATE;
		GPIO_PinState __IN2_STATE;
		TIM_HandleTypeDef *__htim;
		MotorSideTypeDef __side;
		MotorDirectionTypeDef __Dir;
		uint16_t __PowerPWM;
		uint32_t __Channel;
};

#endif /* TLE5205_TLE5205_HPP_ */
