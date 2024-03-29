/*
 * I-BUS.cpp
 *
 *  Created on: Nov 12, 2021
 *      Author: Mateusz
 */
#include "../../RobotDrivers/I-BUS/I-BUS.hpp"

#include "../../RobotDrivers/Configuration.h"
#if defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)
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

#ifdef RESTART_IBUS_UART_DMA

IBus::IBus(UART_HandleTypeDef *ibus_huart, void (*CallEmergencyStop)(void), uint8_t *pRxData, DMA_HandleTypeDef *hdma)
{
	__ibus_huart = ibus_huart;
	__IsConnected_tick = 0;
	__CallEmergencyStop = CallEmergencyStop;
	pData = pRxData;
	__hdma = hdma;
}

#else

IBus::IBus(UART_HandleTypeDef *ibus_huart, void (*CallEmergencyStop)(void), uint8_t *pRxData)
{
	__ibus_huart = ibus_huart;
	__IsConnected_tick = 0;
	__CallEmergencyStop = CallEmergencyStop;
	pData = pRxData;
}


#endif

/*
 * @brief Processes Rx Data received through Uart, should be called in uart callback
 * @param pRxData pointer to uart rx data buffer
 * @retval None
 */
void IBus::ProcessRxDataCB(uint8_t *pRxData)
{
	uint16_t Checksum = ((uint16_t*)pRxData)[15];
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
		__AxesData[u8Iter] =  ((uint16_t*)pRxData)[u8Iter];
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

#ifdef RESTART_IBUS_UART_DMA
	HAL_UARTEx_ReceiveToIdle_DMA(__ibus_huart, pData, 32);
	__HAL_DMA_DISABLE_IT(__hdma, DMA_IT_HT);
#endif
}

void IBus::ProcessRxDataCB()
{
	uint8_t *pRxData = pData;
	if(pData == 0)
	{

	}
	uint16_t Checksum = ((uint16_t*)pRxData)[15];
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
		__AxesData[u8Iter] =  ((uint16_t*)pRxData)[u8Iter];
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

#ifdef RESTART_IBUS_UART_DMA
	HAL_UARTEx_ReceiveToIdle_DMA(__ibus_huart, pData, 40);
	__HAL_DMA_DISABLE_IT(__hdma, DMA_IT_HT);
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
#endif

