/*
 * SumoControl.hpp
 *
 *  Created on: Nov 17, 2021
 *      Author: Mateusz
 */

#ifndef SERVOCONTROL_SERVOCONTROL_HPP_
#define SERVOCONTROL_SERVOCONTROL_HPP_
#include "Configuration.h"
#include <Vector>

#define UNABLE_TO_ATTACH_SERVO 0xFF

typedef struct{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_PIN;
	uint16_t ServoAngle;
} ServoControlVectTypeDef;


class ServoControl
{
	public:
		ServoControl(TIM_HandleTypeDef *htim);
		HAL_StatusTypeDef SetServoValue(uint8_t ServoNo, uint16_t ServoAngle);
		uint8_t AttachServo(GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN);
		HAL_StatusTypeDef ServoControlCBHalfPulse(void);
		HAL_StatusTypeDef ServoControlCBUpdate(void);
		HAL_StatusTypeDef StartServos(void);
		uint8_t GetNoOfAttchedServos(void);
	private:

		TIM_HandleTypeDef *__htim;
		HAL_StatusTypeDef __InitTim(void);
		std::vector<ServoControlVectTypeDef> __ServoControlVect1;
		std::vector<ServoControlVectTypeDef> __ServoControlVect2;
		std::vector<ServoControlVectTypeDef> __ServoControlVect3;
		uint8_t __NoOfServosAttached;
		uint8_t __CurrentServoSched;

};



#endif /* SERVOCONTROL_SERVOCONTROL_HPP_ */
