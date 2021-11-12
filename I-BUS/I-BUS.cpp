/*
 * I-BUS.cpp
 *
 *  Created on: Nov 12, 2021
 *      Author: Mateusz
 */
#include "I-BUS.hpp"
#include "..\Configuration.h"

#ifndef EMERGENCY_STOP_AXIS
#define EMERGENCY_STOP_AXIS 4
#endif

#ifndef EMERGENCY_STOP_TIME_TRESHOLD_TICK
#define EMERGENCY_STOP_TIME_TRESHOLD_TICK 30
#endif

/*
 * @brief I-BUS class constructor
 * @note Requires CallEmergencyStop function to be passed as argument for security
 * @param ibus_huart uart handle, not used right now
 * @param CallEmergencyStop needs to be a function that stops robot movement
 */
IBus::IBus(UART_HandleTypeDef *ibus_huart, void (*CallEmergencyStop)(void))
{
	__ibus_huart = ibus_huart;
	__IsConnected_tick = 0;
	__CallEmergencyStop = CallEmergencyStop;
}

/*
 * @brief Processes Rx Data received through Uart, should be called in uart callback
 * @param pRxData pointer to uart rx data buffer
 * @retval None
 */
void IBus::ProcessRxDataCB(uint8_t *pRxData)
{
	uint16_t Checksum = (pRxData[31] << 8) | pRxData[30];
	//Confirm data correct
	for(uint8_t u8Iter = 0; u8Iter < 30; u8Iter++)
	{
		Checksum = Checksum + pRxData[u8Iter];
	}
	//update connection tick
	if(Checksum == 0xFFFF && (pRxData[1] - pRxData[0] == 0x20))
	{
		__IsConnected_tick = HAL_GetTick();
	}
	else
	{
		return;
	}
	//swap bytes and add to axesData
	pRxData += 2;
	for(uint8_t u8Iter = 0; u8Iter < 14; u8Iter++)
	{
		//Move bytes into place and copy values to __axesData
		__AxesData[u8Iter] =  ((uint16_t*)pData)[u8Iter];
		//Substract 1000 from values to get range from 0 to 1000
		__AxesData[u8Iter] -= 1000;
	}
	//call emergency stop if enabled
#ifdef EMERGENCY_STOP_AXIS
	if(__AxesData[EMERGENCY_STOP_AXIS] > 500)
	{
		__CallEmergencyStop();
	}
#endif
}
/*
 * @brief Should be called periodically if EMERGENCY_STOP_AT_CONNECTION_LOSS is enabled
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef IBus::GetConnectionStatus(void)
{

	if((HAL_GetTick() - __IsConnected_tick) > EMERGENCY_STOP_TIME_TRESHOLD_TICK)
	{
#ifdef EMERGENCY_STOP_AT_CONNECTION_LOSS
		__CallEmergencyStop();
#endif
		return HAL_ERROR;
	}
	else
	{
		return HAL_OK;
	}

}
/*
 * @briefReturns axis value
 * @param Axis Axis number to return value of
 * @retval Axis value
 */
uint16_t IBus::GetAxisValue(uint8_t Axis)
{
	return __AxesData[Axis];
}

