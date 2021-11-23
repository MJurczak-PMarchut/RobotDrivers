/*
 * MotorControl.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */
#include "MotorControl.hpp"
#if (defined(UART_USES_DMA) or defined(UART_USES_IT) or defined(UART_USES_WAIT)) or (defined(SPI_USES_DMA) or defined(SPI_USES_IT) or defined(SPI_USES_WAIT))

uint8_t MCInterface::__Instantiated_sides = 0;

#endif
