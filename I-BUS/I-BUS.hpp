/*
 * I-BUS.hpp
 *
 *  Created on: Nov 12, 2021
 *      Author: Mateusz
 */

#ifndef SUMODRIVERS_I_BUS_I_BUS_HPP_
#define SUMODRIVERS_I_BUS_I_BUS_HPP_
#include "../Configuration.h"
#if defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)

#ifdef __cplusplus
class IBus{
	public:
#ifdef RESTART_IBUS_UART_DMA
		IBus(UART_HandleTypeDef *ibus_huart, void (*CallEmergencyStop)(void), uint8_t *pRxData, DMA_HandleTypeDef *hdma);
#else
		IBus(UART_HandleTypeDef *ibus_huart, void (*CallEmergencyStop)(void), uint8_t *pRxData);
#endif
		uint16_t GetAxisValue(uint8_t Axis);
		void ProcessRxDataCB(uint8_t *pRxData);
		void ProcessRxDataCB();
		HAL_StatusTypeDef GetConnectionStatus(void);
	private:
		UART_HandleTypeDef *__ibus_huart;
		uint16_t __AxesData[14];
		void (*__CallEmergencyStop)(void);
		uint32_t __IsConnected_tick;
		uint8_t *pData;
#ifdef RESTART_IBUS_UART_DMA
		DMA_HandleTypeDef *__hdma;
#endif
};
#else
	#error("IBus can be used in Cpp code only for now")
	void IBus(UART_HandleTypeDef *ibus_huart, void (*CallEmergencyStop)(void));
	uint16_t GetAxisValue(uint8_t u8Axis);
	void ProcessRxDataCB(uint8_t *pRxData);
	HAL_StatusTypeDef GetConnectionStatus(void);

#endif
#endif
#endif /* SUMODRIVERS_I_BUS_I_BUS_HPP_ */
