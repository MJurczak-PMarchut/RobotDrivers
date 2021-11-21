/*
 * RobotSpecificDefines.hpp
 *
 *  Created on: 21 lis 2021
 *      Author: jurcz
 */

#ifndef ROBOTSPECIFICDEFINES_HPP_
#define ROBOTSPECIFICDEFINES_HPP_
#include "Configuration.h"

#ifdef ROBOT_IS_FIDOL

	#define SERVO06_Pin GPIO_PIN_2
	#define SERVO06_GPIO_Port GPIOE
	#define SERVO07_Pin GPIO_PIN_3
	#define SERVO07_GPIO_Port GPIOE
	#define SERVO08_Pin GPIO_PIN_4
	#define SERVO08_GPIO_Port GPIOE
	#define SERVO15_Pin GPIO_PIN_5
	#define SERVO15_GPIO_Port GPIOE
	#define SERVO10_Pin GPIO_PIN_2
	#define SERVO10_GPIO_Port GPIOB
	#define SERVO09_Pin GPIO_PIN_7
	#define SERVO09_GPIO_Port GPIOE
	#define SERVO05_Pin GPIO_PIN_8
	#define SERVO05_GPIO_Port GPIOE
	#define SERVO04_Pin GPIO_PIN_9
	#define SERVO04_GPIO_Port GPIOE
	#define SERVO03_Pin GPIO_PIN_10
	#define SERVO03_GPIO_Port GPIOE
	#define SERVO14_Pin GPIO_PIN_12
	#define SERVO14_GPIO_Port GPIOB
	#define SERVO13_Pin GPIO_PIN_13
	#define SERVO13_GPIO_Port GPIOB
	#define SERVO12_Pin GPIO_PIN_14
	#define SERVO12_GPIO_Port GPIOB
	#define SERVO11_Pin GPIO_PIN_15
	#define SERVO11_GPIO_Port GPIOA
	#define SERVO16_Pin GPIO_PIN_10
	#define SERVO16_GPIO_Port GPIOC
	#define SERVO17_Pin GPIO_PIN_11
	#define SERVO17_GPIO_Port GPIOC
	#define SERVO00_Pin GPIO_PIN_12
	#define SERVO00_GPIO_Port GPIOC
	#define SERVO01_Pin GPIO_PIN_0
	#define SERVO01_GPIO_Port GPIOD
	#define SERVO02_Pin GPIO_PIN_1
	#define SERVO02_GPIO_Port GPIOD

//#define SERVO00 0
//#define SERVO01 1
//#define SERVO02 2
//#define SERVO03 3
//#define SERVO04 4
//#define SERVO05 5
//#define SERVO06 6
//#define SERVO07 7
//#define SERVO08 8
//#define SERVO09 9
//#define SERVO10 10
//#define SERVO11 11
//#define SERVO12 12
//#define SERVO13 13
//#define SERVO14 14
//#define SERVO15 15
//#define SERVO16 16
//#define SERVO17 17
//
//struct ServoData
//{
//
//} sServoData;
#endif

#endif /* ROBOTSPECIFICDEFINES_HPP_ */
