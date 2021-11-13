/*
 * MotorControl.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */
#include "MotorControl.hpp"

typedef struct {
	uint8_t used_flag;
	uint8_t used_comm_interface;
	MotorSideTypeDef used_motor_side;
	MCInterface *mcint;
}ActiveMotorControllersTypeDef;


//Maximum of 2 active motor controllers of the same type
ActiveMotorControllersTypeDef __ActiveMotorControllers[2];

MCInterface::MCInterface(MotorSideTypeDef side, UART_HandleTypeDef *huart)
{
	__thisController.used_flag = 0xAA;
	__thisController.used_comm_interface = 0xAA;
	__thisController.used_motor_side = side;
}

MCInterface::MCInterface(MotorSideTypeDef side, SPI_HandleTypeDef *hspi)
{
	__thisController.used_flag = 0xAA;
	__thisController.used_comm_interface = 0xBB;
	__thisController.used_motor_side = side;
}

HAL_StatusTypeDef MCInterface::__CheckIfControllerAvailable(void)
{
	uint8_t u8Iter;
	for(u8Iter = 0; u8Iter < 2; u8Iter++)
	{
		//Check if controller is used
		if(__ActiveMotorControllers[u8Iter].used_flag == 0xAA)
		{
			if((__ActiveMotorControllers[u8Iter].used_comm_interface != __thisController.used_comm_interface) ||
			   (__ActiveMotorControllers[u8Iter].used_motor_side == __thisController.used_motor_side))
			{
				return HAL_ERROR;
			}
		}
	}
	if(u8Iter > 1)
	{
		return HAL_ERROR;
	}
	__ActiveMotorControllers[u8Iter].used_flag = 0xAA;
	__ActiveMotorControllers[u8Iter].used_comm_interface = __thisController.used_comm_interface;
	__ActiveMotorControllers[u8Iter].used_motor_side = __thisController.used_motor_side;
	__ActiveMotorControllers[u8Iter].mcint = this;
	__thisController.array_index = u8Iter;
	return HAL_OK;
}

HAL_StatusTypeDef MCInterface::CheckIfControllerInitializedOk(void)
{
	return (__ActiveMotorControllers[__thisController.array_index].mcint == 0)? HAL_ERROR : HAL_OK ;
}
