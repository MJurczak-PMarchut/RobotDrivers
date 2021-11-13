/*
 * MotorControl.hpp
 *
 *  Created on: 12 lis 2021
 *      Author: Mateusz
 */
#include "stm32f4xx_hal.h"
#ifndef SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_
#define SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_

typedef enum {MOTOR_LEFT, MOTOR_RIGHT} MotorSideTypeDef;
typedef enum {VESC_CONTROLLER, l9960T_CONTROLLER} ControllerTypeTypeDef;
typedef enum {MOTOR_DIR_FORWARD, MOTOR_DIR_BACKWARD} MotorDirectionTypeDef;


typedef struct {
	uint8_t used_flag;
	uint8_t used_comm_interface;
	MotorSideTypeDef used_motor_side;
	uint8_t array_index;
}ActiveMotorControllerTypeDef;

class MCInterface{
	public:
		MCInterface(MotorSideTypeDef side, UART_HandleTypeDef *huart);
		MCInterface(MotorSideTypeDef side, SPI_HandleTypeDef *hspi);
		virtual HAL_StatusTypeDef SetMotorPowerPWM(uint16_t PowerPWM) = 0;
		virtual HAL_StatusTypeDef SetMotorDirection(MotorDirectionTypeDef Dir) = 0;
		virtual HAL_StatusTypeDef SetMaxCurrent(uint32_t MaxCurrent_mA) = 0;
		virtual HAL_StatusTypeDef Disable() = 0;
		HAL_StatusTypeDef CheckIfControllerInitializedOk(void);
		HAL_StatusTypeDef EmergencyStop(void);
	private:
		HAL_StatusTypeDef __CheckIfControllerAvailable(void);
		ActiveMotorControllerTypeDef __thisController;

};



#endif /* SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_ */
