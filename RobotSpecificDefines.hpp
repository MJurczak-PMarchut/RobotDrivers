/*
 * RobotSpecificDefines.hpp
 *
 *  Created on: 21 lis 2021
 *      Author: Mateusz
 */

#ifndef ROBOTSPECIFICDEFINES_HPP_
#define ROBOTSPECIFICDEFINES_HPP_
#include "Configuration.h"

#ifdef ROBOT_IS_FIDOL

#define SERVO03_Pin GPIO_PIN_2
#define SERVO03_GPIO_Port GPIOE
#define SERVO04_Pin GPIO_PIN_3
#define SERVO04_GPIO_Port GPIOE
#define SERVO05_Pin GPIO_PIN_4
#define SERVO05_GPIO_Port GPIOE
#define SERVO06_Pin GPIO_PIN_5
#define SERVO06_GPIO_Port GPIOE
#define SERVO07_Pin GPIO_PIN_2
#define SERVO07_GPIO_Port GPIOB
#define SERVO08_Pin GPIO_PIN_7
#define SERVO08_GPIO_Port GPIOE
#define SERVO09_Pin GPIO_PIN_8
#define SERVO09_GPIO_Port GPIOE
#define SERVO10_Pin GPIO_PIN_9
#define SERVO10_GPIO_Port GPIOE
#define SERVO11_Pin GPIO_PIN_10
#define SERVO11_GPIO_Port GPIOE
#define SERVO12_Pin GPIO_PIN_12
#define SERVO12_GPIO_Port GPIOB
#define SERVO13_Pin GPIO_PIN_13
#define SERVO13_GPIO_Port GPIOB
#define SERVO14_Pin GPIO_PIN_14
#define SERVO14_GPIO_Port GPIOB
#define SERVO15_Pin GPIO_PIN_15
#define SERVO15_GPIO_Port GPIOA
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
#define INOUT_Pin GPIO_PIN_8
#define INOUT_GPIO_Port GPIOB

#elif defined(ROBOT_IS_MINISUMO)

	#define MOTOR_LEFT_TIM_CHANNEL TIM_CHANNEL_1
	#define MOTOR_RIGHT_TIM_CHANNEL TIM_CHANNEL_3
	#define MOTOR_LEFT_TIMER htim3
	#define MOTOR_RIGHT_TIMER htim4



	#define MOTOR_NDIS_OFFSET 5
	#define MOTOR_LEFT_NDIS_ENABLED (1<<5)
	#define MOTOR_RIGHT_NDIS_ENABLED (1<<6)

	#define TOF_GPIO_1_Pin GPIO_PIN_5
	#define TOF_GPIO_1_GPIO_Port GPIOE
	#define XSHUT_1_Pin GPIO_PIN_6
	#define XSHUT_1_GPIO_Port GPIOE
	#define START_SW_Pin GPIO_PIN_0
	#define START_SW_GPIO_Port GPIOB
	#define TOF_GPIO_3_Pin GPIO_PIN_9
	#define TOF_GPIO_3_GPIO_Port GPIOE
	#define XSHUT_3_Pin GPIO_PIN_10
	#define XSHUT_3_GPIO_Port GPIOE
	#define TOF_GPIO_4_Pin GPIO_PIN_15
	#define TOF_GPIO_4_GPIO_Port GPIOE
	#define XSHUT_4_Pin GPIO_PIN_10
	#define XSHUT_4_GPIO_Port GPIOB
	#define MD_CS_2_Pin GPIO_PIN_11
	#define MD_CS_2_GPIO_Port GPIOD
	#define MD_DIS_2_Pin GPIO_PIN_12
	#define MD_DIS_2_GPIO_Port GPIOD
	#define MD_IN2_DIR_B_Pin GPIO_PIN_13
	#define MD_IN2_DIR_B_GPIO_Port GPIOD
	#define MD_IN1_PWM_B_Pin GPIO_PIN_14
	#define MD_IN1_PWM_B_GPIO_Port GPIOD
	#define MD_NDIS_Pin GPIO_PIN_15
	#define MD_NDIS_GPIO_Port GPIOD
	#define MD_IN1_PWM_A_Pin GPIO_PIN_6
	#define MD_IN1_PWM_A_GPIO_Port GPIOC
	#define MD_IN2_DIR_A_Pin GPIO_PIN_7
	#define MD_IN2_DIR_A_GPIO_Port GPIOC
	#define MD_DIS_1_Pin GPIO_PIN_8
	#define MD_DIS_1_GPIO_Port GPIOA
	#define MD_CS_1_Pin GPIO_PIN_9
	#define MD_CS_1_GPIO_Port GPIOA
	#define TOF_GPIO_2_Pin GPIO_PIN_11
	#define TOF_GPIO_2_GPIO_Port GPIOA
	#define XSHUT_2_Pin GPIO_PIN_12
	#define XSHUT_2_GPIO_Port GPIOA
	#define XSHUT_5_Pin GPIO_PIN_11
	#define XSHUT_5_GPIO_Port GPIOC
	#define TOF_GPIO_6_Pin GPIO_PIN_12
	#define TOF_GPIO_6_GPIO_Port GPIOC
	#define XSHUT_6_Pin GPIO_PIN_0
	#define XSHUT_6_GPIO_Port GPIOD

#endif

#endif /* ROBOTSPECIFICDEFINES_HPP_ */
