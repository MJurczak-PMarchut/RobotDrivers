/*
 * miniTomi.cpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 */
#include "cmsis_os2.h"
#include "osapi.h"
#include "portmacro.h"
#include "stm32h7xx_hal.h"
#include <cmath>
#include <stdio.h>
#include <string.h>
#ifdef ROBOT_MT_V4
#include "Algo.hpp"
#include "HeadingPID.hpp"
#include "vl53l1x.hpp"
#include "L9960T.hpp"
#include "DirtyLogger.hpp"
#include <string>
#include "lsm6dso.hpp"
#include "Adafruit_SSD1327.h"
#include "osapi_mortal_thread.h"
#include "semphr.h"
// Shared helpers/objects split out of this file (display drawing, boundingBox,
// PID_CalibrateHeading, FightAlgorithm, LookForOpponent2, Calibrate, and the
// file-scope objects those share). Also defines the V4 tuning constants
// OLED_I2C_ADDRESS/FULL_SPEED/MANOUVER_SPEED/ROTATION_TIMEOUT/MAX_SIG_PER_SPAD/
// HEADING_PID_DEADBAND_DEG used below.
#include "MiniTomiV4Helpers.hpp"

// Qwiic/STEMMA QT SSD1327 module - I2C only (no reset line broken out on the
// Qwiic connector), address confirmed on this board's silkscreen/datasheet.
#define OLED_WIDTH 128
#define OLED_HEIGHT 128
// The OLED dirty-tracking grid (GRAYOLED_WINDOW_COLS x GRAYOLED_WINDOW_ROWS) is configured via
// build flags in CMakePresets.json (the MiniTomiV4 preset's CUBE_CMAKE_CXX_FLAGS), NOT here:
// the SSD1327/GrayOLED driver is a separate translation unit and never sees #defines from this
// file. Default (no flags) is 1x1 = one whole-screen window; this build uses 2x2 = 4 windows.


#define INIT_TIMEOUT_MS  6000


#define SEARCH_STRATEGY_CHANGE_TIME  3000

// DETECTION_DISTANCE, FULL_SPEED, MANOUVER_SPEED, ROTATION_TIMEOUT, MAX_SIG_PER_SPAD and
// HEADING_PID_DEADBAND_DEG live in MiniTomiV4Helpers.hpp (shared with the helpers TU).

#define HEADING_PID_KP 0.03f
#define HEADING_PID_KI 0.04f
#define HEADING_PID_KD 0.00065f

#define HEADING_PID_OUTPUT_LIMIT 0.75f

// Near a 180-deg error the shortest turn direction is ambiguous and yaw noise flips the
// wrapped error between +-180 every sample. Within this band of the antipode the PID
// sticks to its previous turn direction; outside it the plain shortest-way error rules,
// so the worst case is turning (180 + this) deg instead of (180 - this).
#define HEADING_ANTIPODE_HYST_DEG 10.0f

#define LED2_ON HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_OFF HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern CommManager MainCommManager;

uint32_t means[4] = {0};
uint16_t sens;
//config
L9960T MOTOR_CONTROLLERS[] = {
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, LEFT_MOTOR_PWM_CHANNEL, LEFT_MOTOR_TIMER_PTR, LEFT_MOTOR_INVERTED_PWM, true, false),
		[MOTOR_RIGHT] = L9960T(MOTOR_RIGHT, &hspi2, &MainCommManager, RIGHT_MOTOR_PWM_CHANNEL, RIGHT_MOTOR_TIMER_PTR, RIGHT_MOTOR_INVERTED_PWM, true, false)};


LSM6DSO IMU = LSM6DSO(&MainCommManager, &hspi2);

#ifdef DEBUG_SCREEN
// No reset pin wired (Qwiic connector is power+I2C only) - matches Adafruit_SSD1327's
// own default of no reset pin for I2C-only wiring.
Adafruit_SSD1327 display = Adafruit_SSD1327(OLED_WIDTH, OLED_HEIGHT, &Wire);

// display.display() is flushed off the Robot task by DisplayThread; see DisplayMortalThread
// in MiniTomiV4Helpers.hpp for why the (100ms+) I2C flush is isolated onto its own task.
// RequestUpdate() is the non-blocking entry point other code should call.
DisplayMortalThread DisplayThread;
#endif
VL53L1X Sensors[] = {
		VL53L1X(FRONT_LEFT, &MainCommManager, &hi2c1),
		VL53L1X(BACK, &MainCommManager, &hi2c1),
		VL53L1X(FRONT_RIGHT, &MainCommManager, &hi2c1)
}; //, VL53L1X(FRONT, &MainCommManager, &hi2c1)};

volatile bool lastDetOnLeft = false;
DirtyLogger logger = DirtyLogger(&retSD, SDPath, &SDFatFS, &SDFile);

// Display drawing helpers (_init_*/_stat_draw_bounding_boxes) and the boundingBox class
// now live in MiniTomiV4Helpers.{hpp,cpp}.

/*
 * Algorithm
 */
Robot::Robot():MortalThread(tskIDLE_PRIORITY, 4096, "Main Algo task")
{
}

volatile uint16_t maxVal_perSPAD[2] = {2000, 2000};

// Heading-hold PID; PID_CalibrateHeading() may overwrite the gains at runtime.
HeadingPID HeadingController = HeadingPID(HEADING_PID_KP, HEADING_PID_KI, HEADING_PID_KD,
		HEADING_PID_OUTPUT_LIMIT, HEADING_ANTIPODE_HYST_DEG);

boundingBox StatsBox[4] = {boundingBox(0, "TOF"), boundingBox(1, "Gyro"),
	boundingBox(2, "Strat"), boundingBox(3, "misc")};

void Robot::begin(void)
{
	was_running = true;
	flash_period_ms = 100;
	// HAL_StatusTypeDef imuret;
	logger.Init();
	logger.StartTimestamp();
	//Enable power
	//Enable power
	logger.Log("--- INIT Started ---", LOGLEVEL_INFO);

	HAL_GPIO_WritePin(EXT_LDO_EN_GPIO_Port, EXT_LDO_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NCS1_GPIO_Port, NCS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NCS2_GPIO_Port, NCS2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

	HAL_Delay(100);
	HAL_GPIO_WritePin(EXT_LDO_EN_GPIO_Port, EXT_LDO_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(250);
#ifdef DEBUG_SCREEN
	logger.Log("--- Display Started ---", LOGLEVEL_INFO);

	_init_display();
	display.println("MiniSumo Init Started");
	_init_pg_draw_boundry();
	_init_pg_draw_percentage(10);
	display.setCursor(0, 20);
	_init_log_print_message("---------------------");
	_init_log_print_message("SD Initialized");
	DisplayThread.RequestUpdate();
#endif
	IMU.Init();
	HAL_Delay(100);
	IMU.StartCalibrationOrientation();
#ifdef DEBUG_SCREEN
	_init_pg_draw_percentage(20);
	_init_log_print_message("IMU Initialized");
	DisplayThread.RequestUpdate();
#endif
	HAL_Delay(250);
#ifdef DEBUG_SCREEN
	_init_pg_draw_percentage(45);
#endif
	IMU.CalibrateOrientation();
#ifdef DEBUG_SCREEN
	_init_log_print_message("IMU Calibration");
	DisplayThread.RequestUpdate();
#endif
	HAL_Delay(200);
#ifdef DEBUG_SCREEN
	_init_log_print_message("Start ToF task");
	_init_pg_draw_percentage(75);
	DisplayThread.RequestUpdate();
#endif
	ToF_Sensor::StartSensorTask();
	logger.Log("--- Sensor Task Started---", LOGLEVEL_INFO);
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
	MOTOR_CONTROLLERS[MOTOR_LEFT].Init(0);
	logger.Log("--- Motors Started ---", LOGLEVEL_INFO);

	while(MOTOR_CONTROLLERS[MOTOR_LEFT].CheckIfControllerInitializedOk() != HAL_OK)
	{taskYIELD();}

	MOTOR_CONTROLLERS[MOTOR_RIGHT].Init(0);
	flash_period_ms = 300;
	while(MOTOR_CONTROLLERS[MOTOR_RIGHT].CheckIfControllerInitializedOk() != HAL_OK)
	{taskYIELD();}
	MCInterface::run();
	
	flash_period_ms = 2000;
	while(ToF_Sensor::CheckInitializationCplt() != true)
	{taskYIELD();}
#ifdef DEBUG_SCREEN
	_init_log_print_message("ToF OK");
	_init_pg_draw_percentage(100);
#endif

	logger.Log("--- INIT Completed ---", LOGLEVEL_INFO);
	
	MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
#ifdef DEBUG_SCREEN
	_init_log_print_message("INIT COMPLETED");
	_init_log_print_message("---------------------");
	DisplayThread.RequestUpdate();
	vTaskDelay(10);

	StatsBox[0].attach_stat("L");
	StatsBox[0].attach_stat("P");
	StatsBox[0].attach_stat("T");
#endif
}

void prepare_sensor_data(char *pData, VL53L1X Sensors[])
{
}


void Robot::SetTargetHeading(float targetAngleDeg){
	set_angle = targetAngleDeg;
	HeadingController.SetTarget(targetAngleDeg);
}

// PID_CalibrateHeading() (heading-PID relay auto-tune) now lives in MiniTomiV4Helpers.cpp.

// isOutOfBounds(), GetValidatedDistances(), isOpponentDetected(), StateName() and
// CheckStartCondition() (plus the lineDetOnLeft/lineDetOnRight edge latches and the
// FightAlgorithm/LookForOpponent2/Calibrate sub-routines) now live in
// MiniTomiV4Helpers.cpp.

void Robot::SetFlashPeriodMS(uint16_t flashPeriod)
{
	this->flash_period_ms = flashPeriod;
}

UBaseType_t heap_min_size;

void Robot::loop(void)
{
	// static float angle = 0;
	static State_t eFSM_state = CALIBRATE;
	static bool direction_ack = false;
	eFSM_state = CheckStartCondition(this, eFSM_state);
	static TickType_t last_detection_tick = 0;
	// Collision handling: the IMU driver latches sudden accel/yaw-rate jumps (impacts)
	// that our own motor-driven moves don't produce.
	if((eFSM_state == LOOK_FOR_OPPONENT_1 || eFSM_state == LOOK_FOR_OPPONENT_2) && IMU.IsCollisionDetected())
	{
		IMU.ClearCollisionDetected();
		char collision_message[100];
		sprintf(collision_message, "Collision in state %s", StateName(eFSM_state));
		logger.Log(collision_message, LOGLEVEL_INFO);
		// Hit while searching means the opponent has contact but our sensors don't see them:
		// spin in place (search 1) to reacquire instead of driving on blindly.
		last_detection_tick = xTaskGetTickCount();
		eFSM_state = LOOK_FOR_OPPONENT_1;
	}
	else if(eFSM_state == FIGHT && IMU.IsCollisionDetected())
	{
		logger.Log("----  Contact with opponent detected while fighting ----", LOGLEVEL_INFO);
		IMU.ClearCollisionDetected();  // We expected a collision
	}
	switch(eFSM_state)
	{
		case CALIBRATE:
		{
			Calibrate();
			eFSM_state = WAIT_FOR_START;
			#ifdef DEBUG_SCREEN
			display.setCursor(0,0);
			display.println("State: WAIT FOR START");
			#endif
		}
		break;
		case WAIT_FOR_START:
		{
			isOpponentDetected();
			IMU.ResetAngularOrientation();
			last_detection_tick = xTaskGetTickCount();
			MCInterface::SetMotorsPower(0, 0);
			if(!direction_ack)
			{
				if(IMU.GetAngularOrientationForAxis(2) > 60)
				{
					lastDetOnLeft = false;
					direction_ack = true;
				}
				else if (IMU.GetAngularOrientationForAxis(2) < -60) {
					lastDetOnLeft = true;
					direction_ack = true;
				}
			}
			IMU.ClearCollisionDetected();
		}
		break;
		case FIGHT:
		{
			direction_ack = true;
			if(isOpponentDetected())
			{
			//First fight
				FightAlgorithm(this);
			}
			else
			{
				eFSM_state = LOOK_FOR_OPPONENT_1;
				last_detection_tick = xTaskGetTickCount();
			}
			//Then check OOB condition
			if(isOutOfBounds())
			{
				eFSM_state = OUT_OF_BOUNDS;
			}
		}
		break;
		case OUT_OF_BOUNDS:
		{
			//Check where we crossed the line
			LED2_ON;
			TickType_t startTicks = xTaskGetTickCount();
			MCInterface::SetMotorsPower(0, 0);
			char oob_message[100];
			sprintf(oob_message, " Out Of Bounds: line %s%s, escape turn %s",
					(lineDetOnLeft)? "L" : "", (lineDetOnRight)? "R" : "",
					(lastDetOnLeft)? "LEFT" : "RIGHT");
			logger.Log(oob_message, LOGLEVEL_INFO);
			startTicks = xTaskGetTickCount() - startTicks;
			startTicks = (startTicks < 10)? startTicks : 10;
			vTaskDelay(10 - startTicks);
			//back for a sec
			MCInterface::SetMotorsPower(-MANOUVER_SPEED, -MANOUVER_SPEED);
			vTaskDelay(200);
			if(lastDetOnLeft)
				MCInterface::SetMotorsPower(-MANOUVER_SPEED, MANOUVER_SPEED);
			else
				MCInterface::SetMotorsPower(MANOUVER_SPEED, -MANOUVER_SPEED);
			vTaskDelay(100);
			eFSM_state = FIGHT;
			LED2_OFF;
		}
		break;
		case LOOK_FOR_OPPONENT_1:
		{
			if(isOpponentDetected())
			{
				eFSM_state = FIGHT;
			}
			else
			{
				// static float TargetLookingAngle = 0;
				// float MovementDir = (!lastDetOnLeft) ? 1 : -1;  // -1 for left, 1 for right
				this->SetFlashPeriodMS(1000);
				// Move in 4deg increments
				// if(fabsf(HeadingPID::WrapAngleDeg180((float)IMU.GetAngularOrientationForAxis(2) - TargetLookingAngle)) < 2){
				// 	TargetLookingAngle = HeadingPID::WrapAngleDeg180(TargetLookingAngle + (10 * MovementDir));
				// 	this->SetTargetHeading(TargetLookingAngle);
				// }
				double SteeringTrim = (!lastDetOnLeft) ? MANOUVER_SPEED : -MANOUVER_SPEED;
				// float SteeringTrim = this->GetHeadingCorrection();
				MCInterface::SetMotorsPower(SteeringTrim, -SteeringTrim);

				// Wait 4s before changing strategy
				if((xTaskGetTickCount() - last_detection_tick) < SEARCH_STRATEGY_CHANGE_TIME)
				{
					eFSM_state = LOOK_FOR_OPPONENT_1;
				}
				else
				{
					eFSM_state = LOOK_FOR_OPPONENT_2;
				}
			}
			eFSM_state = (isOutOfBounds())? OUT_OF_BOUNDS : eFSM_state;
		}
		break;
		case LOOK_FOR_OPPONENT_2:
		{
			eFSM_state = (isOpponentDetected()) ? FIGHT : LookForOpponent2(this);
		}
		break;
		case NOP:
			vTaskDelay(1);
			break;
		default:
			eFSM_state = WAIT_FOR_START;
			break;
	}
	// Log every FSM transition so match logs show the full decision timeline
	static State_t last_logged_state = CALIBRATE;
	if(last_logged_state != eFSM_state)
	{
		char state_message[100];
		sprintf(state_message, "FSM: %s -> %s", StateName(last_logged_state), StateName(eFSM_state));
		logger.Log(state_message, LOGLEVEL_INFO);
		last_logged_state = eFSM_state;
	}
	//check if we can move
	taskYIELD();
}

void Robot::end(void)
{
//		Error_Handler();
// Facilitate reset
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
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
	if(GPIO_Pin == IMU_INT1_Pin || GPIO_Pin == IMU_INT2_Pin){
		IMU.InterruptCallback(GPIO_Pin);
	}
//	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
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
float currentAngle = 0;
void Robot::PeriodicCheckCall(void)
{
	static uint8_t call_count = 0;
	static uint16_t flash_counter = flash_period_ms/2;
	static uint16_t current_fc = flash_period_ms;
	static uint32_t start_tick = xTaskGetTickCount();
	static uint8_t diver = 0;
	static uint8_t reset_counter = 0;

	UNUSED(heap_min_size);
	currentAngle = (float)IMU.GetAngularOrientationForAxis(2);
	if(current_fc != flash_period_ms)
	{
		flash_counter = flash_period_ms/2;
		current_fc = flash_period_ms;
	}
	if(!isInitCompleted())
	{
		if((xTaskGetTickCount() - start_tick) > INIT_TIMEOUT_MS)
		{
			if(!this->isRunning() && was_running)
			{
				this->run();
			}
			else{
				if(reset_counter >= 1)
				{
					logger.Sync();
					NVIC_SystemReset();
				}
				reset_counter++;
				this->forceKill();
				ToF_Sensor::KillSensors();
				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
				start_tick = xTaskGetTickCount();
			}
		}

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

				if(diver == 5){
					logger.Sync();
//					HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
					
				}
				#ifdef DEBUG_SCREEN
				DisplayThread.RequestUpdate();
				#endif
				diver = (diver >= 5)? 0: diver+1;
			}
			break;
		default:
			break;
	}
	call_count = (call_count >= 100)? 0: call_count+1;
	if(flash_counter == 0){
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	}
	flash_counter = (flash_counter == 0)? flash_period_ms/2: flash_counter-1;

// PID calculations
	this->power_correction = HeadingController.Update((float)IMU.GetAngularOrientationForAxis(2));
// Heap check
	
}

void Robot::PeriodCB(void)
{
	MOTOR_CONTROLLERS[MOTOR_LEFT].SoftPWMCB_period();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SoftPWMCB_period();
}
#endif
