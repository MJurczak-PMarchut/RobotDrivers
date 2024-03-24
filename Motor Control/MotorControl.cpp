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

MCInterface::MCInterface()
{
	_MCInterfacePointers[NoOfControllers] = this;
	NoOfControllers++;
}


#ifdef USES_RTOS

TaskHandle_t MCInterface::xHandle = {NULL};
bool MCInterface::_isRunning = false;

void MCInterface::run(void)
{
	BaseType_t xReturned;
	if(NoOfControllers > 0)
	{
		xReturned = xTaskCreate(MCInterface::_check_state,
								"MC Status check",
								512,
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

void MCInterface::RunStateCheck(void)
{
	if(!_isRunning){
		_check_state((void*)1);
	}
}


void MCInterface::_check_state(void* pvParam)
{
	static uint32_t ommit_delay = (uint32_t) pvParam;
	for(uint8_t u8Iter = 0; u8Iter < NoOfControllers; u8Iter++)
	{
		if(_MCInterfacePointers[u8Iter]->CheckIfControllerInitializedOk() == HAL_OK){
			_MCInterfacePointers[u8Iter]->CheckControllerState();
		}
	}
	if(ommit_delay == 0){
		vTaskDelay(50);
	}
}
#endif

#endif
