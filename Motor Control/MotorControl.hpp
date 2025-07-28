/*
 * MotorControl.hpp
 *
 *  Created on: 12 lis 2021
 *      Author: Mateusz
 */
#ifndef SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_
#define SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_
#include "CommManager.hpp"
#include "../../RobotDrivers/Configuration.h"
#include "osapi.h"

#if (defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)) or (defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT))

#include <queue>

typedef enum {MOTOR_LEFT = 0, MOTOR_RIGHT = 1} MotorSideTypeDef;
typedef enum {NONE = 0, VESC_CONTROLLER, l9960T_CONTROLLER} ControllerTypeTypeDef;
typedef enum {MOTOR_DIR_FORWARD = 0, MOTOR_DIR_BACKWARD = 1} MotorDirectionTypeDef;
typedef enum {MOTOR_DISABLED, MOTOR_ENABLED} MotorEnabledTypeDef;


typedef struct {
	uint8_t used_flag;
	uint8_t used_comm_interface;
	MotorSideTypeDef used_motor_side;
	uint8_t array_index;
}ActiveMotorControllerTypeDef;

class MCInterface{
	public:
		MCInterface(MotorSideTypeDef side);
		~MCInterface(){};
		static HAL_StatusTypeDef SetMotorsPower(float PowerL, float PowerR);
		HAL_StatusTypeDef SetMotorPower(float Power);
		virtual HAL_StatusTypeDef SetMotorPowerPWM(uint16_t PowerPWM) = 0;
		virtual HAL_StatusTypeDef SetMotorDirection(MotorDirectionTypeDef Dir) = 0;
		virtual HAL_StatusTypeDef Disable(void) = 0;
		virtual HAL_StatusTypeDef Enable(void) = 0;
		virtual HAL_StatusTypeDef EmergencyStop(void) = 0;
		virtual HAL_StatusTypeDef CheckControllerState(void) = 0;
		virtual HAL_StatusTypeDef CheckIfControllerInitializedOk(void) = 0;
		static void run(void);
		static void RunStateCheck(void);
	protected:
		static uint8_t NoOfControllers;
		static uint8_t __Instantiated_sides;
		MotorSideTypeDef __side;
	private:
		static bool _isRunning;
		static void _check_state(void* pvParam);
		static MCInterface*  _MCInterfacePointers[2];
		static TaskHandle_t xHandle;
};


#endif

#endif /* SUMODRIVERS_MOTOR_CONTROL_MOTORCONTROL_HPP_ */
