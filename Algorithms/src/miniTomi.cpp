/*
 * miniTomi.cpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 */
#ifdef ROBOT_MT_V1
#include "miniTomi.hpp"
#include "vl53l5cx.hpp"
#include "L9960T.hpp"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern CommManager MainCommManager;

//config
static  L9960T MOTOR_CONTROLLERS[] = {
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, TIM_CHANNEL_1, &htim3),
		[MOTOR_RIGHT] = L9960T(MOTOR_RIGHT, &hspi2, &MainCommManager, TIM_CHANNEL_3, &htim4)};

static VL53L5CX Sensors[] ={ VL53L5CX(FRONT_LEFT, &MainCommManager, &hi2c1), VL53L5CX(FRONT_LEFT, &MainCommManager, &hi2c1) };

miniTomi::miniTomi():MortalThread(tskIDLE_PRIORITY, 1024)
{
}

void miniTomi::begin(void)
{
	ToF_Sensor::StartSensorTask();
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	MOTOR_CONTROLLERS[MOTOR_LEFT].Init(0);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Init(0);
	while(MOTOR_CONTROLLERS[MOTOR_LEFT].CheckIfControllerInitializedOk() != HAL_OK)
	{taskYIELD();}
	while(MOTOR_CONTROLLERS[MOTOR_RIGHT].CheckIfControllerInitializedOk() != HAL_OK)
	{taskYIELD();}
	while(ToF_Sensor::CheckInitializationCplt() != true)
	{taskYIELD();}
	MOTOR_CONTROLLERS[MOTOR_LEFT].Enable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Enable();
}

void miniTomi::loop(void)
{
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(100);
	//algorithm goes here
}

void miniTomi::end(void)
{
	Error_Handler();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == TOF_GPIO_6_Pin || GPIO_Pin == TOF_GPIO_5_Pin || GPIO_Pin == TOF_GPIO_4_Pin || GPIO_Pin == TOF_GPIO_3_Pin || GPIO_Pin == TOF_GPIO_2_Pin)
	{
		if(ToF_Sensor::CheckInitializationCplt())
		{
			ToF_Sensor::EXTI_Callback_func(GPIO_Pin);
		}
	}
}

#endif
