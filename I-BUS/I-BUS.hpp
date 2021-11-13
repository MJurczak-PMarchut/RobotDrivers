/*
 * I-BUS.hpp
 *
 *  Created on: Nov 12, 2021
 *      Author: Mateusz
 */

#ifndef SUMODRIVERS_I_BUS_I_BUS_HPP_
#define SUMODRIVERS_I_BUS_I_BUS_HPP_
#include "../Configuration.h"

#ifdef MCU_STM32F4
#include "stm32f4xx_hal.h"
#elif MCU_STM32H7
#include "stm32h7xx_hal.h"
#endif

#ifdef __cplusplus
class IBus{
	public:
		IBus(UART_HandleTypeDef *ibus_huart, void (*CallEmergencyStop)(void));
		uint16_t GetAxisValue(uint8_t Axis);
		void ProcessRxDataCB(uint8_t *pRxData);
		HAL_StatusTypeDef GetConnectionStatus(void);
	private:
		UART_HandleTypeDef *__ibus_huart;
		uint16_t __AxesData[14];
		void (*__CallEmergencyStop)(void);
		uint32_t __IsConnected_tick;
};
#else
	#error("IBus can be used in Cpp code only for now")
	void IBus(UART_HandleTypeDef *ibus_huart, void (*CallEmergencyStop)(void));
	uint16_t GetAxisValue(uint8_t u8Axis);
	void ProcessRxDataCB(uint8_t *pRxData);
	HAL_StatusTypeDef GetConnectionStatus(void);

#endif

#endif /* SUMODRIVERS_I_BUS_I_BUS_HPP_ */
