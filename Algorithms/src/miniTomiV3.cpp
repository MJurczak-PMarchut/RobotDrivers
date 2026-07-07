/*
 * miniTomi.cpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 */
#include "osapi.h"
#include "portmacro.h"
#include <stdio.h>
#ifdef ROBOT_MT_V3
#include "Algo.hpp"
#include "vl53l1x.hpp"
#include "L9960T.hpp"
#include "DirtyLogger.hpp"
#include <string>
#include "lsm6dso.hpp"

#define INIT_TIMEOUT_MS  6000


#define DETECTION_DISTANCE 350
#define FULL_SPEED 0.85
#define MANOUVER_SPEED 0.65

#define SEARCH_STRATEGY_CHANGE_TIME  3000
#define ROTATION_TIMEOUT 150

#define MAX_SIG_PER_SPAD	8000

#define HEADING_PID_KP 0.02f
#define HEADING_PID_KI 0.0005f
#define HEADING_PID_KD 0.01f
#define HEADING_PID_OUTPUT_LIMIT 0.65f


// Errors smaller than this are treated as zero, so sensor noise / motor response lag
// doesn't make the robot hunt back and forth for a negligible heading difference.
#define HEADING_PID_DEADBAND_DEG 15.0f

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
static  L9960T MOTOR_CONTROLLERS[] = {
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, LEFT_MOTOR_PWM_CHANNEL, LEFT_MOTOR_TIMER_PTR, LEFT_MOTOR_INVERTED_PWM, true, false),
		[MOTOR_RIGHT] = L9960T(MOTOR_RIGHT, &hspi2, &MainCommManager, RIGHT_MOTOR_PWM_CHANNEL, RIGHT_MOTOR_TIMER_PTR, RIGHT_MOTOR_INVERTED_PWM, true, false)};


static LSM6DSO IMU = LSM6DSO(&MainCommManager, &hspi2);

static VL53L1X Sensors[] = {
		VL53L1X(FRONT_LEFT, &MainCommManager, &hi2c1),
		VL53L1X(BACK, &MainCommManager, &hi2c1),
		VL53L1X(FRONT_RIGHT, &MainCommManager, &hi2c1)
}; //, VL53L1X(FRONT, &MainCommManager, &hi2c1)};

volatile bool lastDetOnLeft = false;
bool sensor_detected_item[2]= {false, false};
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
Robot::Robot():MortalThread(tskIDLE_PRIORITY, 4096, "Main Algo task")
{
}

volatile uint16_t maxVal_perSPAD[2] = {2000, 2000};

// Live heading-PID gains; PID_CalibrateHeading() overwrites these at runtime.
static float headingPID_Kp = HEADING_PID_KP;
static float headingPID_Ki = HEADING_PID_KI;
static float headingPID_Kd = HEADING_PID_KD;

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
	IMU.Init();
	HAL_Delay(100);
	IMU.StartCalibrationOrientation();
	HAL_Delay(150);
	IMU.CalibrateOrientation();
	HAL_GPIO_WritePin(EXT_LDO_EN_GPIO_Port, EXT_LDO_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	ToF_Sensor::StartSensorTask();
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
	MOTOR_CONTROLLERS[MOTOR_LEFT].Init(0);
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

	logger.Log("--- INIT Completed ---", LOGLEVEL_INFO);
	MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
}

void prepare_sensor_data(char *pData, VL53L1X Sensors[])
{
}

// Which line sensor(s) fired on the most recent out-of-bounds event; latched here
// because the pins may already read clear by the time the OOB handler logs them.
volatile bool lineDetOnLeft = false;
volatile bool lineDetOnRight = false;

bool isOutOfBounds(void)
{
	bool left = !HAL_GPIO_ReadPin(LDLeft_GPIO_Port, LDLeft_Pin);
	bool right = !HAL_GPIO_ReadPin(LDRight_GPIO_Port, LDRight_Pin);
	if(left || right)
	{
		lineDetOnLeft = left;
		lineDetOnRight = right;
	}
	return (left || right);
}

void WaitForNewData(void)
{	
	static EventBits_t u32_updated_Sensors = 0;
	u32_updated_Sensors = xEventGroupWaitBits(ToF_Sensor::GetEventHandle(), TOF_EVENT_MASK, pdTRUE, pdTRUE, 20);
	UNUSED(u32_updated_Sensors);
}

typedef struct{
	uint16_t raw_left;
	uint16_t raw_right;
	uint16_t left;
	uint16_t right;
}SensorDistances_t;

// Reads both front sensors and replaces readings whose signal is weaker than the
// calibrated threshold with an out-of-detection-range value, so callers only act
// on distances the calibration deems trustworthy.
static SensorDistances_t GetValidatedDistances(void)
{
	SensorDistances_t distances;
	distances.raw_left = Sensors[0].GetDistance();
	distances.raw_right = Sensors[2].GetDistance();
	VL53L1X_Result_t result_left = Sensors[0].GetResult();
	VL53L1X_Result_t result_right = Sensors[2].GetResult();
	distances.left = (result_left.SigPerSPAD > maxVal_perSPAD[0])?
			distances.raw_left: DETECTION_DISTANCE + 100;
	distances.right = (result_right.SigPerSPAD > maxVal_perSPAD[1])?
			distances.raw_right: DETECTION_DISTANCE + 100;
	return distances;
}

bool isOpponentDetected(void)
{
	char sensor_message[100] = {0};
	static bool was_previously_detected= false;

	WaitForNewData();
	SensorDistances_t distances = GetValidatedDistances();
	sprintf(sensor_message, "ToF [0]: %u, [1]: %u ", distances.raw_left, distances.raw_right);
	logger.Log(sensor_message,LOGLEVEL_TRACE);

	sensor_detected_item[0] = distances.left < DETECTION_DISTANCE;
	sensor_detected_item[1] = distances.right < DETECTION_DISTANCE;
	bool detected = sensor_detected_item[0] || sensor_detected_item[1];

	if(was_previously_detected != detected){
		logger.Log((detected)? "Opponent Detected" : "No Oponnent", LOGLEVEL_INFO);
	}

	sprintf(sensor_message, "Orientation x: %.2f, y: %.2f, z:%.2f ", IMU.GetAngularOrientationForAxis(0), IMU.GetAngularOrientationForAxis(1), IMU.GetAngularOrientationForAxis(2));
	logger.Log(sensor_message, LOGLEVEL_TRACE);

	was_previously_detected = detected;
	return detected;
}

typedef enum{CALIBRATE, WAIT_FOR_START, FIGHT, OUT_OF_BOUNDS, LOOK_FOR_OPPONENT_1, LOOK_FOR_OPPONENT_2} State_t;

static const char* StateName(State_t state)
{
	static const char *names[] = {"CALIBRATE", "WAIT_FOR_START", "FIGHT", "OUT_OF_BOUNDS", "LOOK_FOR_OPPONENT_1", "LOOK_FOR_OPPONENT_2"};
	return (state <= LOOK_FOR_OPPONENT_2)? names[state] : "UNKNOWN";
}

State_t  CheckStartCondition(Robot *obj, State_t eFSM_state)
{
	static bool start_condition_prev = false;
	bool start_condition = HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin);
	if(start_condition != start_condition_prev)
	{
		logger.Log((start_condition)? " Start Condition Set" : " Start Condition Reset ", LOGLEVEL_INFO);
	}
	start_condition_prev = start_condition;
	if(eFSM_state ==  CALIBRATE){
		return eFSM_state;
	}
	if(!start_condition)//dummy
	{
		obj->SetFlashPeriodMS(1000);
		MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
		MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
		return WAIT_FOR_START;
	}
	else{
		MOTOR_CONTROLLERS[MOTOR_LEFT].Enable();
		MOTOR_CONTROLLERS[MOTOR_RIGHT].Enable();
		return  (eFSM_state == WAIT_FOR_START)? FIGHT : eFSM_state;
	}
}

void FightAlgorithm(Robot *obj)
{
	bool forward_det;
	SensorDistances_t distances = GetValidatedDistances();
	uint16_t sensor_left = distances.left;
	uint16_t sensor_right = distances.right;
	// detected
	obj->SetFlashPeriodMS(100);
	if(sensor_left > sensor_right){
		forward_det = ((sensor_left - sensor_right) < 60)? true:false;
	}
	else
	{
		forward_det = ((sensor_right - sensor_left) < 60)? true:false;
	}
	const char *fight_move;
	if (forward_det){
		MCInterface::SetMotorsPower(FULL_SPEED, FULL_SPEED);
		fight_move = "ATTACK FWD";
	}
	else if(sensor_left < sensor_right)
	{
		// on the left
		MCInterface::SetMotorsPower(MANOUVER_SPEED, FULL_SPEED);
		lastDetOnLeft = true;
		fight_move = "TURN LEFT";
	}
	else{
		MCInterface::SetMotorsPower(FULL_SPEED, MANOUVER_SPEED);
		lastDetOnLeft = false;
		fight_move = "TURN RIGHT";
	}

	// Log only when the steering decision changes, with the distances that drove it
	static const char *last_fight_move = NULL;
	if(fight_move != last_fight_move)
	{
		char fight_message[100];
		sprintf(fight_message, "Fight: %s L=%u R=%u", fight_move, sensor_left, sensor_right);
		logger.Log(fight_message, LOGLEVEL_INFO);
		last_fight_move = fight_move;
	}
}

// Returns a steering correction in the same range as MCInterface::SetMotorsPower,
// computed from the IMU's Z-axis (yaw) angle vs. targetAngleDeg.
float PID_HeadingCorrection(float targetAngleDeg)
{
	static float integral = 0.0f;
	static float lastError = 0.0f;
	static TickType_t lastTick = 0;
	static float lastTargetAngle = NAN;

	bool targetChanged = (targetAngleDeg != lastTargetAngle);
	if (targetChanged)
	{
		integral = 0.0f;
		lastTargetAngle = targetAngleDeg;
	}

	TickType_t now = xTaskGetTickCount();
	float dt = (lastTick == 0) ? 0.0f : (float)(now - lastTick) / 1000.0f;
	lastTick = now;

	float currentAngle = (float)IMU.GetAngularOrientationForAxis(2);
	float error = targetAngleDeg - currentAngle;

	if (targetChanged)
	{
		lastError = error; // avoid a derivative kick from the previous target's error
	}

	float derivative = (dt > 0.0f) ? (error - lastError) / dt : 0.0f;
	lastError = error;
	float integral_tmp = error * dt;
	integral += integral_tmp;
	float integralLimit = (headingPID_Ki > 0.0f) ? 2.0f * (HEADING_PID_OUTPUT_LIMIT / headingPID_Ki) : 0.0f;
	if (integral > integralLimit)
	{
		integral = integralLimit;
	}
	else if (integral < -integralLimit)
	{
		integral = -integralLimit;
	}

	float output = headingPID_Kp * error + headingPID_Ki * integral + headingPID_Kd * derivative;

	if (output > HEADING_PID_OUTPUT_LIMIT)
	{
		output = HEADING_PID_OUTPUT_LIMIT;
		integral -= integral_tmp; // anti-windup
	}
	else if (output < -HEADING_PID_OUTPUT_LIMIT)
	{
		output = -HEADING_PID_OUTPUT_LIMIT;
		integral -= integral_tmp; // anti-windup
	}

	return output;
}

// Relay (Astrom-Hagglund) auto-tune for the heading PID: drives the motors in a
// bang-bang pattern around the current heading to force a sustained oscillation,
// measures its period/amplitude, and derives Kp/Ki/Kd via the standard Ziegler-Nichols
// relay formulas. Runs for at most maxDurationMs, so it always returns in finite time.
// The robot WILL spin in place while this runs - only call it with the robot on a stand
// or clear of obstacles, never during a match.
void PID_CalibrateHeading(uint32_t maxDurationMs, float relayPower)
{
	const uint8_t targetHalfCycles = 10; // stop early once enough oscillations are captured
	float targetAngle = (float)IMU.GetAngularOrientationForAxis(2);

	TickType_t startTick = xTaskGetTickCount();
	TickType_t lastCrossingTick = startTick;
	float peakAmplitude = 0.0f;
	float amplitudeSum = 0.0f;
	uint32_t halfPeriodSum = 0;
	uint8_t halfCycles = 0;
	bool firstCrossingSeen = false;
	bool relayHigh = true; // which side of the relay we're currently driving

	while ((xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(maxDurationMs) && halfCycles < targetHalfCycles)
	{
		float currentAngle = (float)IMU.GetAngularOrientationForAxis(2);
		float error = targetAngle - currentAngle;

		float absError = (error < 0.0f) ? -error : error;
		if (absError > peakAmplitude)
		{
			peakAmplitude = absError;
		}

		// Hysteresis around the deadband: only flip once the error clearly crosses to the
		// other side, so sensor noise near zero doesn't register as extra spurious cycles.
		bool relayFlipped = false;
		if (relayHigh && error < -HEADING_PID_DEADBAND_DEG)
		{
			relayHigh = false;
			relayFlipped = true;
		}
		else if (!relayHigh && error > HEADING_PID_DEADBAND_DEG)
		{
			relayHigh = true;
			relayFlipped = true;
		}

		if (relayFlipped)
		{
			// Relay flip => half oscillation cycle complete
			TickType_t now = xTaskGetTickCount();
			if (firstCrossingSeen)
			{
				halfPeriodSum += (now - lastCrossingTick);
				amplitudeSum += peakAmplitude;
				halfCycles++;
			}
			firstCrossingSeen = true;
			lastCrossingTick = now;
			peakAmplitude = 0.0f;
		}

		if (relayHigh)
		{
			MCInterface::SetMotorsPower(-relayPower, relayPower);
		}
		else
		{
			MCInterface::SetMotorsPower(relayPower, -relayPower);
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	MCInterface::SetMotorsPower(0.0f, 0.0f);

	if (halfCycles == 0)
	{
		return; // no oscillation captured within maxDurationMs; leave existing gains untouched
	}

	float averageAmplitudeDeg = amplitudeSum / halfCycles;
	float periodS = 2.0f * ((float)halfPeriodSum / halfCycles) / 1000.0f; // half-cycle -> full period

	if (averageAmplitudeDeg <= 0.0f || periodS <= 0.0f)
	{
		return;
	}

	float Ku = (4.0f * relayPower) / (3.14159265f * averageAmplitudeDeg); // ultimate gain
	float Tu = periodS;                                                   // ultimate period

	headingPID_Kp = 0.6f * Ku;
	headingPID_Ki = 1.2f * Ku / Tu;   // Ti = Tu/2  ->  Ki = Kp/Ti = 2*Kp/Tu
	headingPID_Kd = 0.075f * Ku * Tu; // Td = Tu/8  ->  Kd = Kp*Td = Kp*Tu/8
}

void Robot::SetFlashPeriodMS(uint16_t flashPeriod)
{
	this->flash_period_ms = flashPeriod;
}

State_t LookForOpponent2(Robot *obj)
{
	TickType_t start_tick;
	double_t starting_orientation = NAN;
	double_t angularOrientation = NAN;
	obj->SetFlashPeriodMS(500);
	if(isOutOfBounds())
	{
		char search_message[100];
		sprintf(search_message, "Search2: edge hit, backing off, rotate %s", (lastDetOnLeft)? "LEFT" : "RIGHT");
		logger.Log(search_message, LOGLEVEL_INFO);
		// On both Or Only One?
		//move back slightly
		MCInterface::SetMotorsPower(-MANOUVER_SPEED, -MANOUVER_SPEED);
		vTaskDelay(250);
		// move into last det
		starting_orientation = IMU.GetAngularOrientationForAxis(2);
		start_tick = xTaskGetTickCount();
		if(lastDetOnLeft)
			MCInterface::SetMotorsPower(-MANOUVER_SPEED, MANOUVER_SPEED);
		else
			MCInterface::SetMotorsPower(MANOUVER_SPEED, -MANOUVER_SPEED);
		while(xTaskGetTickCount() - start_tick < ROTATION_TIMEOUT)
		{
			angularOrientation = IMU.GetAngularOrientationForAxis(2) - starting_orientation;
			angularOrientation = (angularOrientation > 0)? angularOrientation : -angularOrientation;
			if(angularOrientation >= 60)
			{
				MCInterface::SetMotorsPower(MANOUVER_SPEED, MANOUVER_SPEED);
				break;
			}
			if(isOpponentDetected())
				return FIGHT;
		}
	}
	else
	{
		MCInterface::SetMotorsPower(MANOUVER_SPEED, MANOUVER_SPEED);
	}
	return LOOK_FOR_OPPONENT_2;
}
UBaseType_t heap_min_size;

void Calibrate(void)
{
	TickType_t start_time = xTaskGetTickCount();
	while((xTaskGetTickCount() - start_time) < ROTATION_TIMEOUT){
		xEventGroupWaitBits(ToF_Sensor::GetEventHandle(), TOF_EVENT_MASK, pdTRUE, pdTRUE, 30);
		VL53L1X_Result_t result_left = Sensors[0].GetResult();
		VL53L1X_Result_t result_right = Sensors[2].GetResult();
		maxVal_perSPAD[0] = (maxVal_perSPAD[0] < result_left.SigPerSPAD)? result_left.SigPerSPAD: maxVal_perSPAD[0];
		maxVal_perSPAD[1] = (maxVal_perSPAD[1] < result_right.SigPerSPAD)? result_right.SigPerSPAD: maxVal_perSPAD[1];
	}
	maxVal_perSPAD[0] = maxVal_perSPAD[0]*2;
	maxVal_perSPAD[1] = maxVal_perSPAD[1]*2;
	maxVal_perSPAD[0] = (maxVal_perSPAD[0] < MAX_SIG_PER_SPAD)? maxVal_perSPAD[0]:MAX_SIG_PER_SPAD;
	maxVal_perSPAD[1] = (maxVal_perSPAD[1] < MAX_SIG_PER_SPAD)? maxVal_perSPAD[1]:MAX_SIG_PER_SPAD;
	
	char calib_message[100];
	sprintf(calib_message, "Calib SigPerSPAD thresholds L=%u R=%u", maxVal_perSPAD[0], maxVal_perSPAD[1]);
	logger.Log(calib_message, LOGLEVEL_INFO);
	logger.Log("--- CALIBRATION Completed ---", LOGLEVEL_INFO);
}


void Robot::loop(void)
{
	static State_t eFSM_state = CALIBRATE;
	eFSM_state = CheckStartCondition(this, eFSM_state);
	static TickType_t last_detection_tick = 0;
	// Collision handling: the IMU driver latches sudden accel/yaw-rate jumps (impacts)
	// that our own motor-driven moves don't produce.
	if(eFSM_state == CALIBRATE || eFSM_state == WAIT_FOR_START)
	{
		// Discard jolts from handling/placing the robot before the match
		IMU.ClearCollisionDetected();
	}
	else if(IMU.IsCollisionDetected())
	{
		IMU.ClearCollisionDetected();
		char collision_message[100];
		sprintf(collision_message, "Collision in state %s", StateName(eFSM_state));
		logger.Log(collision_message, LOGLEVEL_INFO);
		// Hit while searching means the opponent has contact but our sensors don't see them:
		// spin in place (search 1) to reacquire instead of driving on blindly.
		if(eFSM_state == LOOK_FOR_OPPONENT_1 || eFSM_state == LOOK_FOR_OPPONENT_2)
		{
			last_detection_tick = xTaskGetTickCount();
			eFSM_state = LOOK_FOR_OPPONENT_1;
		}
	}
	switch(eFSM_state)
	{
		case CALIBRATE:
		{
			Calibrate();
			eFSM_state = WAIT_FOR_START;
		}
		break;
		case WAIT_FOR_START:
		{
			isOpponentDetected();
		}
		break;
		case FIGHT:
		{
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
				this->SetFlashPeriodMS(1000);
				double SteeringTrim = (!lastDetOnLeft) ? MANOUVER_SPEED : -MANOUVER_SPEED;
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

void Robot::PeriodicCheckCall(void)
{
	static uint8_t call_count = 0;
	static uint16_t flash_counter = flash_period_ms/2;
	static uint16_t current_fc = flash_period_ms;
	static uint32_t start_tick = xTaskGetTickCount();
	static uint8_t diver = 0;
	static uint8_t reset_counter = 0;

	UNUSED(heap_min_size);

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

				if(diver == 20){
					logger.Sync();
//					HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
				}
				diver = (diver >= 20)? 0: diver+1;
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
	this->power_correction = PID_HeadingCorrection(this->set_angle);
// Heap check
	
}

void Robot::PeriodCB(void)
{
	MOTOR_CONTROLLERS[MOTOR_LEFT].SoftPWMCB_period();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SoftPWMCB_period();
}
#endif
