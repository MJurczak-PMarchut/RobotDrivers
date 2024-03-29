/*
 * Configuration.h
 *
 *  Created on: Nov 12, 2021
 *      Author: Mateusz
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include "main.h"
#include "SpecialTypes.hpp"

#define EMERGENCY_STOP_AXIS 4
#define EMERGENCY_STOP_TIME_TRESHOLD_TICK 30
#define EMERGENCY_STOP_AT_CONNECTION_LOSS


#define MAX_MESSAGE_NO_IN_QUEUE 100

#define USES_RTOS


#ifdef USES_RTOS
#include "cmsis_os.h"
#endif

//#define MCU_STM32F4
#define MCU_STM32H7

//#define SPI_USES_DMA
#define SPI_USES_IT
//#define SPI_USES_WAIT

#define UART_USES_DMA
//#define UART_USES_IT
//#define UART_USES_WAIT

//#define I2C_USES_DMA
#define I2C_USES_IT
//#define I2C_USES_WAIT

#define CSn_ACTIVE_PIN_STATE GPIO_PIN_RESET

#if defined(SPI_USES_WAIT) || defined(I2C_USES_WAIT) || defined(UART_USES_WAIT)
	#define COMMUNICATION_TIMEOUT HAL_MAX_DELAY
#endif


#ifdef MCU_STM32F4
#include "stm32f4xx_hal.h"
#elif defined(MCU_STM32H7)
#include "stm32h7xx_hal.h"
#endif

#if CSn_ACTIVE_PIN_STATE == GPIO_PIN_RESET
#define CSn_INACTIVE_PIN_STATE GPIO_PIN_SET
#else
#define CSn_INACTIVE_PIN_STATE GPIO_PIN_RESET
#endif

#if !defined(I2C_USES_DMA) && !defined(I2C_USES_IT) && !defined(I2C_USES_WAIT) && !defined(SPI_USES_DMA) && !defined(SPI_USES_IT) && !defined(SPI_USES_WAIT) && !defined(UART_USES_DMA) && !defined(UART_USES_IT) && !defined(UART_USES_WAIT)
#define NO_COMMUNICATION_INTERFACE_DEFINED
#endif

//#define ROBOT_IS_FIDOL
//#define ROBOT_IS_TOMISLAW




#define RESTART_IBUS_UART_DMA

#define MAX_LINE_DETECTOR_COUNT 4

#define GPIO_NUMBER_SUMO (16U)

#endif /* CONFIGURATION_H_ */
