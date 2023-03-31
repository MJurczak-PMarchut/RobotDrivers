/*
 * miniTomi.cpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 */
#ifdef ROBOT_BRCK_V1
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
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, LEFT_MOTOR_PWM_CHANNEL, LEFT_MOTOR_TIMER_PTR),
		[MOTOR_RIGHT] = L9960T(MOTOR_RIGHT, &hspi2, &MainCommManager, RIGHT_MOTOR_PWM_CHANNEL, RIGHT_MOTOR_TIMER_PTR)};

static VL53L5CX Sensors[] ={ VL53L5CX(FRONT_LEFT, &MainCommManager, &hi2c1), VL53L5CX(FRONT_RIGHT, &MainCommManager, &hi2c1), VL53L5CX(FRONT, &MainCommManager, &hi2c1)  };


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
uint16_t data[4] = {0};
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
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
	Sensors[0].SetRotation(ROTATE_270);
	Sensors[1].SetRotation(ROTATE_180);
	Sensors[2].SetRotation(ROTATE_90);
}


//File declarations
typedef enum {WAIT_FOR_START, SERACH_FOR_OPPONENT, BACKOFF_FROM_LINE, CHASE_OPPONENT} RobotState;
RobotState eRobotState = WAIT_FOR_START;

typedef enum {OPPONENT_SLIGHTLY_ON_THE_LEFT, OPPONENT_ON_THE_LEFT, OPPONENT_SLIGHTLY_ON_THE_RIGHT, OPPONENT_ON_THE_RIGHT, OPPONENT_IN_FRONT, OPPONENT_NOT_SEEN} OpponentLocation;
OpponentLocation eOpponentLastLocation = OPPONENT_NOT_SEEN;

uint32_t means[6] = {0};

void MeasureMeans(void);
bool OpponentFound(void);
OpponentLocation LocateOpponent(void);


#define START_CONDITION true

void Robot::loop(void)
{
	//Calculate Distances

	//algorithm goes here
	switch(eRobotState)
	{
		case WAIT_FOR_START:
			eOpponentLastLocation = OPPONENT_NOT_SEEN;
			MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(0);
			MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(0);
			if(START_CONDITION){
				//Start moving here
			}
			else{
				//try to find opponent while still
			}
			eRobotState = (START_CONDITION)?SERACH_FOR_OPPONENT:WAIT_FOR_START;
			break;
		case SERACH_FOR_OPPONENT:
			//	lets spin!!!
			MeasureMeans();
			if(OpponentFound())
			{
				eRobotState = (START_CONDITION)?CHASE_OPPONENT:WAIT_FOR_START;
			}
			else{
				switch(eOpponentLastLocation){
					case OPPONENT_NOT_SEEN:
					case OPPONENT_ON_THE_LEFT:
					case OPPONENT_SLIGHTLY_ON_THE_LEFT:
					case OPPONENT_IN_FRONT:

						MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
						MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
						break;

					case OPPONENT_ON_THE_RIGHT:
					case OPPONENT_SLIGHTLY_ON_THE_RIGHT:
					default:

						MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_BACKWARD);
						MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
						break;
				}
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(OPPONENT_SEARCH_SPEED);
				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(OPPONENT_SEARCH_SPEED);

				eRobotState = (START_CONDITION)?SERACH_FOR_OPPONENT:WAIT_FOR_START;
			}
			break;
		case BACKOFF_FROM_LINE:
			//We do not have line sensor how did we get here?
			eRobotState = (START_CONDITION)?SERACH_FOR_OPPONENT:WAIT_FOR_START;
			break;
		case CHASE_OPPONENT:
			MeasureMeans();
			if(!OpponentFound()){
				eRobotState = (START_CONDITION)?SERACH_FOR_OPPONENT:WAIT_FOR_START;
			}
			else{
				eOpponentLastLocation = LocateOpponent();
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
				switch(eOpponentLastLocation)
				{
					case OPPONENT_ON_THE_LEFT:
						MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(OPPONENT_CHASE_SPEED);
						MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(OPPONENT_CHASE_SPEED/3);
						break;

					case OPPONENT_SLIGHTLY_ON_THE_LEFT:
						MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(OPPONENT_CHASE_SPEED);
						MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(OPPONENT_CHASE_SPEED/2);
						break;

					case OPPONENT_ON_THE_RIGHT:
						MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(OPPONENT_CHASE_SPEED/3);
						MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(OPPONENT_CHASE_SPEED);
						break;

					case OPPONENT_SLIGHTLY_ON_THE_RIGHT:
						MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(OPPONENT_CHASE_SPEED/2);
						MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(OPPONENT_CHASE_SPEED);
						break;

					case OPPONENT_IN_FRONT:
						MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(OPPONENT_CHASE_SPEED);
						MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(OPPONENT_CHASE_SPEED);
						break;
					default:
						break;
				}
			}
			eRobotState = (START_CONDITION)?eRobotState:WAIT_FOR_START;
			break;
		default:
			eRobotState = (START_CONDITION)?SERACH_FOR_OPPONENT:WAIT_FOR_START;
			break;
	}
	taskYIELD();

}

OpponentLocation LocateOpponent(void)
{
	//find the lowest value
	uint8_t lowest_index = 0;
	uint32_t lowest_value = 0xFFFFFF;
	for(uint8_t iter=0; iter < 6; iter++){
		if(means[iter]<lowest_value){
			lowest_index = iter;
			lowest_value = means[iter];
		}
	}
	if(lowest_index == 0){
		return OPPONENT_ON_THE_LEFT;
	}
	else if(lowest_index == 5){
		return OPPONENT_ON_THE_RIGHT;
	}
	else{
		//check if in front
		if((lowest_index == 2) || (lowest_index == 3)){
			if(means[5-lowest_index] - lowest_value < 50){
				return OPPONENT_IN_FRONT;
			}
			else{
				return (lowest_index == 2)? OPPONENT_SLIGHTLY_ON_THE_LEFT: OPPONENT_SLIGHTLY_ON_THE_RIGHT;
			}
		}
		else{
			return (lowest_index == 1)? OPPONENT_SLIGHTLY_ON_THE_LEFT:OPPONENT_SLIGHTLY_ON_THE_RIGHT;
		}
	}
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


bool OpponentFound(void)
{
	for(uint8_t iter=0; iter < 6; iter++)
	{
		if(means[iter] < DETECTION_THRESHOLD)
		{
			return true;
		}
	}
	return false;
}


void MeasureMeans(void)
{
	for(uint8_t iter=0; iter < 6; iter++)
	{
		means[iter] = Sensors[iter >> 1].GetDataFromSensor(((3-(iter&0x3))<<1)&0x3, 1); //left and down
		means[iter] += Sensors[iter >> 1].GetDataFromSensor((((3-(iter&0x3))<<1)&0x3) + 1, 1); //right, down
		means[iter] += Sensors[iter >> 1].GetDataFromSensor(((3-(iter&0x3))<<1)&0x3, 2); //left, up
		means[iter] += Sensors[iter >> 1].GetDataFromSensor((((3-(iter&0x3))<<1)&0x3) + 1, 2); //right, up
		means[iter] = means[iter] >> 2;
	}
}

#endif
