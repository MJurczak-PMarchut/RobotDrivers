/*
 * MotorControl.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */
#include "../../RobotDrivers/Motor Control/MotorControl.hpp"
#if (defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)) or (defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT))

uint8_t MCInterface::__Instantiated_sides = 0;
MCInterface*  MCInterface::_MCInterfacePointers[2] = {0};
uint8_t MCInterface::NoOfControllers = 0;

MCInterface::MCInterface(MotorSideTypeDef side):  __side{side}
{
	_MCInterfacePointers[side] = this;
	NoOfControllers++;
}

HAL_StatusTypeDef MCInterface::SetMotorPower(float Power)
{
	uint8_t ret = HAL_OK;
	MotorDirectionTypeDef dir = (Power>0)? MOTOR_DIR_FORWARD :
			(Power == 0)? __motorDir : MOTOR_DIR_BACKWARD;
	float _power = (Power > 1)? 1:
			(Power < -1)? -1:
			(Power < 0)?  -Power: Power;
	uint16_t _PWM = _power * 999;
	_PWM = (_PWM > 999)? 999:_PWM;
	ret |= this->SetMotorDirection(dir);
	ret |= this->SetMotorPowerPWM(_PWM);
	return (HAL_StatusTypeDef)ret;
}

HAL_StatusTypeDef MCInterface::SetMotorsPower(float PowerL, float PowerR)
{
	uint8_t ret = HAL_OK;
	ret |= _MCInterfacePointers[MOTOR_LEFT]->SetMotorPower(PowerL);
	ret |= _MCInterfacePointers[MOTOR_RIGHT]->SetMotorPower(PowerR);
	return (HAL_StatusTypeDef)ret;
}

#ifdef USES_RTOS

TaskHandle_t MCInterface::xHandle = {NULL};
bool MCInterface::_isRunning = false;

void MCInterface::run(void)
{
	BaseType_t xReturned;
	if(_isRunning)
	{
		return;
	}
	if(NoOfControllers > 0)
	{
		xReturned = xTaskCreate(MCInterface::_check_state,
								"MC Status check",
								1024,
								NULL,
								tskIDLE_PRIORITY,
								&xHandle);
	}
	if(xReturned != pdTRUE)
	{
		Error_Handler();
	}
	_isRunning = true;
}


void MCInterface::_check_state(void* pvParam)
{
	static uint32_t ommit_delay = (uint32_t) pvParam;
	uint32_t call_count = 0;
	vTaskDelay(500);
	while(true){
		for(uint8_t u8Iter = 0; u8Iter < NoOfControllers; u8Iter++)
		{
			if(_MCInterfacePointers[u8Iter]->CheckIfControllerInitializedOk() == HAL_OK){
				_MCInterfacePointers[u8Iter]->CheckControllerState();
			}
		}
		if(ommit_delay == 0){
			vTaskDelay(10);
		}
		call_count++;
	}
}
#endif

#endif
