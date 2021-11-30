/*
 * MotorControl.hpp
 *
 *  Created on: 12 lis 2021
 *      Author: Mateusz
 */
#ifndef SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_
#define SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_
#include "Configuration.h"
#include "CommManager.hpp"
#if (defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)) or (defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT))

#include <queue>

typedef enum {MOTOR_LEFT = 0, MOTOR_RIGHT = 1} MotorSideTypeDef;
typedef enum {VESC_CONTROLLER, l9960T_CONTROLLER} ControllerTypeTypeDef;
typedef enum {MOTOR_DIR_FORWARD = 0, MOTOR_DIR_BACKWARD} MotorDirectionTypeDef;
typedef enum {MOTOR_DISABLED, MOTOR_ENABLED} MotorEnabledTypeDef;


typedef struct {
	uint8_t used_flag;
	uint8_t used_comm_interface;
	MotorSideTypeDef used_motor_side;
	uint8_t array_index;
}ActiveMotorControllerTypeDef;

class MCInterface{
	public:
		virtual HAL_StatusTypeDef SetMotorPowerPWM(uint16_t PowerPWM) = 0;
		virtual HAL_StatusTypeDef SetMotorDirection(MotorDirectionTypeDef Dir) = 0;
//		virtual HAL_StatusTypeDef SetMaxCurrent(uint32_t MaxCurrent_mA) = 0;
		virtual HAL_StatusTypeDef Disable(void) = 0;
		virtual HAL_StatusTypeDef Enable(void) = 0;
		virtual HAL_StatusTypeDef EmergencyStop(void) = 0;
	private:
	protected:
		static uint8_t __Instantiated_sides;
};


#endif

#endif /* SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_ */
