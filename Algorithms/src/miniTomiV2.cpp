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
uint16_t sens;
//config
static  L9960T MOTOR_CONTROLLERS[] = {
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, LEFT_MOTOR_PWM_CHANNEL, LEFT_MOTOR_TIMER_PTR, LEFT_MOTOR_INVERTED_PWM, true),
		[MOTOR_RIGHT] = L9960T(MOTOR_RIGHT, &hspi2, &MainCommManager, RIGHT_MOTOR_PWM_CHANNEL, RIGHT_MOTOR_TIMER_PTR, RIGHT_MOTOR_INVERTED_PWM, true)};

static VL53L5CX Sensors[] ={ VL53L5CX(FRONT, &MainCommManager, &hi2c1),  VL53L5CX(FRONT_LEFT, &MainCommManager, &hi2c1),  VL53L5CX(FRONT_RIGHT, &MainCommManager, &hi2c1)};

//static DirtyLogger logger = DirtyLogger(&retSD, SDPath, &SDFatFS, &SDFile);
/*
 * static functions definitions
 */


/*
 * static functions implementation
 */



/*
 * Algorithm
 */
Robot::Robot():MortalThread(tskIDLE_PRIORITY, 4096, "Main Algo task")
{
}

void Robot::begin(void)
{
//	logger.Init();
	//Enable power
	HAL_GPIO_WritePin(EXT_LDO_EN_GPIO_Port, EXT_LDO_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	ToF_Sensor::StartSensorTask();
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
	MOTOR_CONTROLLERS[MOTOR_LEFT].Init(0);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Init(0);
	while(MOTOR_CONTROLLERS[MOTOR_LEFT].CheckIfControllerInitializedOk() != HAL_OK)
	{taskYIELD();}
	while(MOTOR_CONTROLLERS[MOTOR_RIGHT].CheckIfControllerInitializedOk() != HAL_OK)
	{taskYIELD();}
	while(ToF_Sensor::CheckInitializationCplt() != true)
	{taskYIELD();}
	MCInterface::run();
//	logger.Log("Starting loop", LOGLEVEL_INFO);

	MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();






}

void prepare_sensor_data(char *pData, VL53L5CX Sensors[])
{
	uint8_t sensor_index;
	uint16_t distance_array[16];
	for(sensor_index = 0; sensor_index < ToF_Sensor::GetNoOfSensors(); sensor_index++)
	{
		distance_array[0] = Sensors[sensor_index].GetDataFromSensor(0, 0);
		distance_array[1] = Sensors[sensor_index].GetDataFromSensor(1, 0);
		distance_array[2] = Sensors[sensor_index].GetDataFromSensor(2, 0);
		distance_array[3] = Sensors[sensor_index].GetDataFromSensor(3, 0);
		distance_array[4] = Sensors[sensor_index].GetDataFromSensor(0, 1);
		distance_array[5] = Sensors[sensor_index].GetDataFromSensor(1, 1);
		distance_array[6] = Sensors[sensor_index].GetDataFromSensor(2, 1);
		distance_array[7] = Sensors[sensor_index].GetDataFromSensor(3, 1);
		distance_array[8] = Sensors[sensor_index].GetDataFromSensor(0, 2);
		distance_array[9] = Sensors[sensor_index].GetDataFromSensor(1, 2);
		distance_array[10] = Sensors[sensor_index].GetDataFromSensor(2, 2);
		distance_array[11] = Sensors[sensor_index].GetDataFromSensor(3, 2);
		distance_array[12] = Sensors[sensor_index].GetDataFromSensor(0, 3);
		distance_array[13] = Sensors[sensor_index].GetDataFromSensor(1, 3);
		distance_array[14] = Sensors[sensor_index].GetDataFromSensor(2, 3);
		distance_array[15] = Sensors[sensor_index].GetDataFromSensor(3, 3);

		sprintf(&pData[strlen(pData)], "\nSensorNumber: %d\n%d %d %d %d\n%d %d %d %d\n%d %d %d %d\n%d %d %d %d\n", (sensor_index),
				distance_array[0], distance_array[1], distance_array[2], distance_array[3],
				distance_array[4], distance_array[5], distance_array[6], distance_array[7],
				distance_array[8], distance_array[9], distance_array[10], distance_array[11],
				distance_array[12], distance_array[13], distance_array[14], distance_array[15]);
	}
}

void Robot::loop(void)
{
//	char message[100];
	static uint16_t speed = 0;
	UNUSED(speed);
	char message_data[400] = {0};
	EventBits_t u32_updated_Sensors;
	UNUSED(u32_updated_Sensors);
	if(!HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin)){
	}
	else{
	}
	u32_updated_Sensors = xEventGroupWaitBits(ToF_Sensor::GetEventHandle(), TOF_EVENT_MASK, pdTRUE, pdTRUE, 1000);
	prepare_sensor_data(message_data, Sensors);
//	logger.Log(message_data, LOGLEVEL_INFO);

}

void Robot::end(void)
{
	Error_Handler();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(ToF_Sensor::CheckInitializationCplt()){
		if (GPIO_Pin == TOF_INT4_Pin)
		{
			ToF_Sensor::EXTI_Callback_func(GPIO_Pin);
		}
		if(GPIO_Pin == TOF_INT3_Pin){
			ToF_Sensor::EXTI_Callback_func(GPIO_Pin);
		}
		if(GPIO_Pin == TOF_INT5_Pin){
			ToF_Sensor::EXTI_Callback_func(GPIO_Pin);
		}
	}
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

void Robot::PeriodicCheckCall(void)
{
	static uint8_t call_count = 0;
	static uint8_t diver = 0;
	if(!isInitCompleted()){
		return;
	}
	// we could use mod, but that might not be fast for architectures other than ARM
	switch(call_count)
	{
		case 25:
			{
//				MCInterface::RunStateCheck();
			}
			break;
		case 50:
			{
//				MCInterface::RunStateCheck();
			}
			break;
		case 75:
			{
//				MCInterface::RunStateCheck();
			}
			break;
		case 100:
			{
//				MCInterface::RunStateCheck();

				if(diver == 20){
//					logger.Sync();
					HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
				}
				diver = (diver >= 20)? 0: diver+1;
			}
			break;
		default:
			break;
	}
	call_count = (call_count >= 100)? 0: call_count+1;
}

void Robot::PeriodCB(void)
{
	MOTOR_CONTROLLERS[MOTOR_LEFT].SoftPWMCB_period();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SoftPWMCB_period();
}
#endif
