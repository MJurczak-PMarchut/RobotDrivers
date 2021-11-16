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
	this->__CheckIfControllerAvailable(side, 0xAA);
}

MCInterface::MCInterface(MotorSideTypeDef side, SPI_HandleTypeDef *hspi, uint16_t CS_Pin, GPIO_TypeDef *GPIOx)
{
	this->__CheckIfControllerAvailable(side, 0xBB);
}

HAL_StatusTypeDef MCInterface::__CheckIfControllerAvailable(MotorSideTypeDef side, uint8_t used_comm_interface)
{
	uint8_t u8Iter;
	uint8_t freespace = 0xFF;
	for(u8Iter = 0; u8Iter < 2; u8Iter++)
	{
		//Check if controller is used
		if(__ActiveMotorControllers[u8Iter].used_flag == 0xAA)
		{
			if((__ActiveMotorControllers[u8Iter].used_comm_interface != used_comm_interface) ||
			   (__ActiveMotorControllers[u8Iter].used_motor_side == side))
			{
				return HAL_ERROR;
			}
		}
		else if(freespace == 0xFF)
		{
			freespace = u8Iter;
		}
	}
	if(freespace == 0xFF)
	{
		return HAL_ERROR;
	}
	__ActiveMotorControllers[u8Iter].used_flag = 0xAA;
	__ActiveMotorControllers[u8Iter].used_comm_interface = used_comm_interface;
	__ActiveMotorControllers[u8Iter].used_motor_side = side;
	__ActiveMotorControllers[u8Iter].mcint = this;
	this->index = freespace;
	return HAL_OK;
}

HAL_StatusTypeDef MCInterface::CheckIfControllerAttachedOk(void)
{
	return (__ActiveMotorControllers[this->index].mcint == 0)? HAL_ERROR : HAL_OK ;
}