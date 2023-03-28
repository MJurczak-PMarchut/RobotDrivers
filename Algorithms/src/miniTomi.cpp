/*
 * miniTomi.cpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 */
#ifdef ROBOT_MT_V1
#include "Algo.hpp"
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

static VL53L5CX Sensors[] ={ VL53L5CX(FRONT_LEFT, &MainCommManager, &hi2c1), VL53L5CX(FRONT_RIGHT, &MainCommManager, &hi2c1) };


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
	MCInterface::run();
	MOTOR_CONTROLLERS[MOTOR_LEFT].Enable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Enable();
}

void Robot::loop(void)
{
	uint32_t means[4] = {0};
	static bool lastDetOnLeft = false;
	//check if we can move
	if(false)//dummy
	{
		vTaskDelay(1);
		return;
	}
	//algorithm goes here

	//Get mean senors measurement
	for(uint8_t iter=0; iter < 4; iter++)
	{
		means[iter] = Sensors[iter >> 1].GetDataFromSensor(((3-iter)<<1)&0x3, 1); //left and down
		means[iter] += Sensors[iter >> 1].GetDataFromSensor((((3-iter)<<1)&0x3) + 1, 1); //right, down
		means[iter] += Sensors[iter >> 1].GetDataFromSensor(((3-iter)<<1)&0x3, 2); //left, up
		means[iter] += Sensors[iter >> 1].GetDataFromSensor((((3-iter)<<1)&0x3) + 1, 2); //right, up
		means[iter] = means[iter] >> 2;
	}
	// Check if opponent is in front
	if(((means[1]+means[2]) >> 1) < 250)
	{
		//if detected object in front at less than 250mm
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
		if(means[1] < means[2])
		{
			lastDetOnLeft = true;
			//Opponent on the left
			if((means[2] - means[1]) > 100)
			{
				//strongly on the left
				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(750);
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(999);
			}
			else
			{
				//Weakly on the left, drive forward
				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(999);
			}
		}
		else
		{
			//Opponent on the right
			lastDetOnLeft = false;
			if((means[1] - means[2]) > 100)
			{
				//strongly on the right
				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(750);
			}
			else
			{
				//Weakly on the right, drive forward
				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(999);
			}
		}
	}
	else if(((means[0] + means[3]) >> 1) < 250)
	{
		if(means[0] < means[3])
		{
			//opponent on the left
			lastDetOnLeft = true;
			MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
			MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
		}
		else
		{
			//opponent on the right
			lastDetOnLeft = false;
			MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
			MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_BACKWARD);
		}
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(999);
	}
	else if(means[0] < 50)
	{
		//Opponent is on our left, very close
		lastDetOnLeft = true;
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(750);
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(999);

	}
	else if(means[3] < 50)
	{
		//Opponent is on our right, very close
		lastDetOnLeft = false;
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(750);

	}
	else
	{
		//opponent not detected
		if(lastDetOnLeft)
		{
			MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
			MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);

		}
		else
		{
			MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
			MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_BACKWARD);
		}
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(999);
	}
	taskYIELD();

}

void Robot::end(void)
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
