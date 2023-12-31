/*
 * miniTomi.cpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 */
#ifdef ROBOT_MT_V2
#include "Algo.hpp"
#include "vl53l5cx.hpp"
#include "L9960T.hpp"
#include "DirtyLogger.hpp"
#include <string>

#define FULL_SPEED 600
#define MANOUVER_SPEED 350

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern CommManager MainCommManager;
uint32_t means[4] = {0};
//config
static  L9960T MOTOR_CONTROLLERS[] = {
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, LEFT_MOTOR_PWM_CHANNEL, LEFT_MOTOR_TIMER_PTR, LEFT_MOTOR_INVERTED_PWM, true),
		[MOTOR_RIGHT] = L9960T(MOTOR_RIGHT, &hspi2, &MainCommManager, RIGHT_MOTOR_PWM_CHANNEL, RIGHT_MOTOR_TIMER_PTR, RIGHT_MOTOR_INVERTED_PWM, true)};


static DirtyLogger logger = DirtyLogger(&retSD, SDPath, &SDFatFS, &SDFile);
/*
 * static functions definitions
 */


/*
 * static functions implementation
 */



/*
 * Algorithm
 */
Robot::Robot():MortalThread(tskIDLE_PRIORITY, 1024)
{
}

void Robot::begin(void)
{

	logger.Init();
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	MOTOR_CONTROLLERS[MOTOR_LEFT].Init(0);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Init(0);
	while(MOTOR_CONTROLLERS[MOTOR_LEFT].CheckIfControllerInitializedOk() != HAL_OK)
	{taskYIELD();}
	while(MOTOR_CONTROLLERS[MOTOR_RIGHT].CheckIfControllerInitializedOk() != HAL_OK)
	{taskYIELD();}
	MCInterface::run();
	logger.Log("Starting loop\n", false);

	MOTOR_CONTROLLERS[MOTOR_LEFT].Enable();



}

void Robot::loop(void)
{
	char message[100];
	static uint16_t speed = 0;
	sprintf(message, "Set Motor Power at %d\n", speed);
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(speed);
	logger.Log(message, false);
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
	speed = (speed >= 999)?0:speed + 100;
	HAL_Delay(250);
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
	HAL_Delay(250);

}

void Robot::end(void)
{
	Error_Handler();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			MOTOR_CONTROLLERS[MOTOR_LEFT].SoftPWMCB_pulse();
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			MOTOR_CONTROLLERS[MOTOR_RIGHT].SoftPWMCB_pulse();
	}
}

void Robot::PeriodCB(void)
{
	MOTOR_CONTROLLERS[MOTOR_LEFT].SoftPWMCB_period();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SoftPWMCB_period();
}
#endif
