/*
 * L9960T.hpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#ifndef SUMODRIVERS_L9960T_L9960T_HPP_
#define SUMODRIVERS_L9960T_L9960T_HPP_
#include "MotorControl.hpp"

class L9960T : public MCInterface{
	public:
		L9960T(MotorSideTypeDef side, SPI_HandleTypeDef *hspi, uint16_t CS_Pin, GPIO_TypeDef);
		HAL_StatusTypeDef AttachPWMTimerAndChannel(TIM_HandleTypeDef *htim, uint32_t Channel);
		HAL_StatusTypeDef SetMotorPowerPWM(uint16_t PowerPWM);
		HAL_StatusTypeDef SetMotorDirection(MotorDirectionTypeDef Dir);
		HAL_StatusTypeDef SetMaxCurrent(uint32_t MaxCurrent_mA);
		HAL_StatusTypeDef Disable();
		HAL_StatusTypeDef CheckIfControllerInitializedOk(void);
	private:

};


#endif /* SUMODRIVERS_L9960T_L9960T_HPP_ */
