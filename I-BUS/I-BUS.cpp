/*
 * I-BUS.cpp
 *
 *  Created on: Nov 12, 2021
 *      Author: Mateusz
 */
#include "I-BUS.hpp"
#include "Configuration.h"

#ifndef EMERGENCY_STOP_AXIS
#define EMERGENCY_STOP_AXIS 4
#endif

#ifndef EMERGENCY_STOP_TIME_TRESHOLD_TICK
#define EMERGENCY_STOP_TIME_TRESHOLD_TICK 30
#endif


IBus::IBus(UART_HandleTypeDef *ibus_huart, void (*CallEmergencyStop)(void))
{
	__ibus_huart = ibus_huart;
	__IsConnected_tick = 0;
	__CallEmergencyStop = CallEmergencyStop;
}

void IBus::ProcessRxDataCB(uint8_t *pRxData)
{
	uint16_t Checksum = (pData[31] << 8) | pData[30];
	//Confirm data correct
	for(uint8_t u8Iter = 0; u8Iter < 30; u8Iter++)
	{
		Checksum = Checksum + pData[u8Iter];
	}
	if(Checksum == 0xFFFF && (pData[1] - pData[0] == 0x20))
	{
		__IsConnected_tick = HAL_GetTick();
	}
	else
	{
		return;
	}
	//swap bytes and add to axesData
	pData += 2;
	for(uint8_t u8Iter = 0; u8Iter < 30; u8Iter = u8Iter + 2)
	{
		__axesData[u8Iter >> 1] =  (pData[u8Iter]) | (pData[u8Iter + 1] << 8);
		__axesData[u8Iter >> 1] -= 1000;
	}
	if(axesData[EMERGENCY_STOP_AXIS] > 500)
	{
		__CallEmergencyStop();
	}
}

HAL_StatusTypeDef IBus::GetConnectionStatus(void)
{
#ifdef EMERGENCY_STOP_AT_CONNECTION_LOSS
	if((HAL_GetTick() - __IsConnected_tick) > EMERGENCY_STOP_TIME_TRESHOLD_TICK)
	{
		__CallEmergencyStop();
		return HAL_ERROR;
	}
	else
	{
		return HAL_OK;
	}
#else

	return ((HAL_GetTick() - __IsConnected_tick) < EMERGENCY_STOP_TIME_TRESHOLD_TICK);
#endif
}

uint16_t IBus::GetAxisValue(uint8_t Axis)
{
	return __AxesData[Axis];
}

