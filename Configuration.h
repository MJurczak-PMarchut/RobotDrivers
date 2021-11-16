/*
 * Configuration.h
 *
 *  Created on: Nov 12, 2021
 *      Author: Mateusz
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#define EMERGENCY_STOP_AXIS 4
#define EMERGENCY_STOP_TIME_TRESHOLD_TICK 30
#define EMERGENCY_STOP_AT_CONNECTION_LOSS


#define MAX_MESSAGE_NO_IN_QUEUE 5


#define MCU_STM32F4
//#define MCU_STM32H7

#define SPI_USES_DMA
//#define SPI_USES_IT
//#define SPI_USES_WAIT

#define UART_USES_DMA
//#define UART_USES_IT
//#define UART_USES_WAIT

#define I2C_USES_DMA
//#define I2C_USES_IT
//#define I2C_USES_WAIT


#if defined(SPI_USES_WAIT) or defined(I2C_USES_WAIT) or defined(UART_USES_WAIT)
	#define COMMUNICATION_TIMEOUT HAL_MAX_DELAY
#endif

#endif /* CONFIGURATION_H_ */
