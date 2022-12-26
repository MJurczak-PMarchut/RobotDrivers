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
#ifdef USES_RTOS
: MortalThread(tskIDLE_PRIORITY, 512)
#endif
{
	_MCInterfacePointers[NoOfControllers] = this;
	NoOfControllers++;
}


#ifdef USES_RTOS
void MCInterface::loop()
{
	for(uint8_t u8Iter = 0; u8Iter < NoOfControllers; u8Iter++)
	{
		if(_MCInterfacePointers[u8Iter]->CheckIfControllerInitializedOk() == HAL_OK){
			_MCInterfacePointers[u8Iter]->CheckControllerState();
		}
	}
	sleep(5);
}
#endif

#endif
