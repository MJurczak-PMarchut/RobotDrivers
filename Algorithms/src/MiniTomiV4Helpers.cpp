/*
 * MiniTomiV4Helpers.cpp
 *
 * Implementations of the display helpers, heading-PID relay auto-tuner, and movement
 * sub-routines split out of miniTomiV4.cpp to keep the V4 strategy file readable.
 * See MiniTomiV4Helpers.hpp for the shared declarations. The whole unit is compiled
 * only for the ROBOT_MT_V4 build.
 */
#include "cmsis_os2.h"
#include "osapi.h"
#include "portmacro.h"
#include "stm32h7xx_hal.h"
#include <cmath>
#include <stdio.h>
#include <string.h>
#ifdef ROBOT_MT_V4
#include "MiniTomiV4Helpers.hpp"

// Which line sensor(s) fired on the most recent out-of-bounds event; latched here
// because the pins may already read clear by the time the OOB handler logs them.
volatile bool lineDetOnLeft = false;
volatile bool lineDetOnRight = false;

// Latest per-sensor "opponent within DETECTION_DISTANCE" flags; only isOpponentDetected()
// reads/writes them, so they stay private to this TU.
static bool sensor_detected_item[2] = {false, false};

#ifdef DEBUG_SCREEN
void _init_pg_draw_boundry(void)
{
	display.drawRect(0, 9, 128, 10, SSD1327_WHITE);
}

void _init_pg_draw_percentage(int16_t percentage)
{
	static int16_t drawn_bar = 0;
	int16_t width = (percentage * 1.24) - drawn_bar;
	display.fillRect(drawn_bar + 2, 11, width, 6, SSD1327_WHITE);
}

void _init_log_print_message(const char* str)
{
	static int16_t CurY = 20;
	display.setCursor(0, CurY);
	display.println(str);
	CurY = display.getCursorY();
}

void _init_display(void)
{
	if (!display.begin(OLED_I2C_ADDRESS))
	{
		logger.Log("--- Display begin() FAILED ---", LOGLEVEL_INFO);
	}

	DisplayThread.run();
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1327_WHITE, SSD1327_BLACK);
	display.setCursor(0, 0);
}

void _stat_draw_bounding_boxes()
{
	display.drawRect(0, 9, 128, 128-9, SSD1327_WHITE);
	display.drawFastVLine(63, 9, (128-9),SSD1327_WHITE);
	display.drawFastHLine(0, 63, 128,SSD1327_WHITE);
}
#endif
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

// Blocks up to 30 ticks for a fresh sweep from all ToF sensors before the caller reads
// their distances. Only isOpponentDetected() needs it, so it stays file-local.
static void WaitForNewData(void)
{
	static EventBits_t u32_updated_Sensors = 0;
	u32_updated_Sensors = xEventGroupWaitBits(ToF_Sensor::GetEventHandle(), TOF_EVENT_MASK, pdTRUE, pdTRUE, 30);
	UNUSED(u32_updated_Sensors);
}

// Reads both front sensors and replaces readings whose signal is weaker than the
// calibrated threshold with an out-of-detection-range value, so callers only act
// on distances the calibration deems trustworthy.
SensorDistances_t GetValidatedDistances(void)
{
	SensorDistances_t distances;
	distances.raw_left = Sensors[0].GetDistance();
	distances.raw_rear = Sensors[1].GetDistance();
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
	sprintf(sensor_message, "ToF:\n\r[0]=%u\n\r[1]=%u\n\r", distances.raw_left, distances.raw_right);
	logger.Log(sensor_message,LOGLEVEL_TRACE);
#ifdef DEBUG_SCREEN
	sprintf(sensor_message, "L=%u  ", distances.left);
	StatsBox[0].printstat(sensor_message, "L");


	sprintf(sensor_message, "P=%u  ", distances.right);
	StatsBox[0].printstat(sensor_message, "P");

	sprintf(sensor_message, "T=%u  ", distances.raw_rear);
	StatsBox[0].printstat(sensor_message, "T");
#endif
	sensor_detected_item[0] = distances.left < DETECTION_DISTANCE;
	sensor_detected_item[1] = distances.right < DETECTION_DISTANCE;
	bool detected = sensor_detected_item[0] || sensor_detected_item[1];

	if(was_previously_detected != detected){
		logger.Log((detected)? "Opponent Detected" : "No Oponnent", LOGLEVEL_INFO);
	}

	sprintf(sensor_message, "Orientation:\n\rx=%.2f\n\ry=%.2f\n\rz=%.2f\n\r", IMU.GetAngularOrientationForAxis(0), IMU.GetAngularOrientationForAxis(1), IMU.GetAngularOrientationForAxis(2));
	logger.Log(sensor_message, LOGLEVEL_TRACE);

	was_previously_detected = detected;
	return detected;
}

const char* StateName(State_t state)
{
	static const char *names[] = {"CALIBRATE", "WAIT_FOR_START", "FIGHT", "OUT_OF_BOUNDS", "LOOK_FOR_OPPONENT_1", "LOOK_FOR_OPPONENT_2"};
	return (state <= LOOK_FOR_OPPONENT_2)? names[state] : "UNKNOWN";
}

State_t CheckStartCondition(Robot *obj, State_t eFSM_state)
{
	static bool start_condition_prev = false;
#ifdef DEBUG_SCREEN
	static bool was_enabled = false;
#endif
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
#ifdef DEBUG_SCREEN
		if(was_enabled){
			display.setCursor(42,0);
			display.println("State: Disabled");
		}
#endif
		obj->SetFlashPeriodMS(1000);
		MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
		MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
		return WAIT_FOR_START;
	}
	else{
		if(eFSM_state == WAIT_FOR_START){
#ifdef DEBUG_SCREEN
			display.setCursor(42,0);
			display.println("State: FIGHT!!!");
#endif
		}
		MOTOR_CONTROLLERS[MOTOR_LEFT].Enable();
		MOTOR_CONTROLLERS[MOTOR_RIGHT].Enable();
#ifdef DEBUG_SCREEN
		was_enabled = true;
#endif
		return  (eFSM_state == WAIT_FOR_START)? FIGHT : eFSM_state;
	}
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
		float error = HeadingPID::WrapAngleDeg180(targetAngle - currentAngle);

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

	float tunedKp = 0.6f * Ku;
	float tunedKi = 1.2f * Ku / Tu;   // Ti = Tu/2  ->  Ki = Kp/Ti = 2*Kp/Tu
	float tunedKd = 0.075f * Ku * Tu; // Td = Tu/8  ->  Kd = Kp*Td = Kp*Tu/8
	HeadingController.SetGains(tunedKp, tunedKi, tunedKd);
	char string[100] = {0};
	sprintf(string, "\n\rP= %f\n\rI= %f\n\rD= %f\n\r", tunedKp, tunedKi, tunedKd);
	logger.Log(string, LOGLEVEL_TRACE);
}

void FightAlgorithm(Robot *obj)
{
	static bool forward_det;
	SensorDistances_t distances = GetValidatedDistances();
	uint16_t sensor_left = distances.left;
	uint16_t sensor_right = distances.right;
	// detected
	obj->SetFlashPeriodMS(100);
	bool fwd_det_tmp = (abs(sensor_left - sensor_right) < 60)? true:false;
	// Latch the target heading at the moment the forward-attack decision flips, then hold
	// it for the whole charge so the PID (run by PeriodicCheckCall) steers the robot
	// straight at where it saw the opponent.
	if(fwd_det_tmp != forward_det)
	{
		obj->SetTargetHeading((float)IMU.GetAngularOrientationForAxis(2));
	}
	forward_det = fwd_det_tmp;
	const char *fight_move;
	if (forward_det){
		// Positive error (target > current yaw) needs a CCW turn: left slower, right faster
		// (same convention the relay autotuner and LookForOpponent2 use).
		float heading_correction = obj->GetHeadingCorrection(1-FULL_SPEED);
		MCInterface::SetMotorsPower(FULL_SPEED - heading_correction, FULL_SPEED + heading_correction);
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
		vTaskDelay(350);
		// move into last det
		starting_orientation = IMU.GetAngularOrientationForAxis(2);
		start_tick = xTaskGetTickCount();
		if(lastDetOnLeft)
			MCInterface::SetMotorsPower(-MANOUVER_SPEED, MANOUVER_SPEED);
		else
			MCInterface::SetMotorsPower(MANOUVER_SPEED, -MANOUVER_SPEED);
		while(xTaskGetTickCount() - start_tick < ROTATION_TIMEOUT)
		{
			angularOrientation = HeadingPID::WrapAngleDeg180((float)(IMU.GetAngularOrientationForAxis(2) - starting_orientation));
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
#ifdef DEBUG_SCREEN
	display.clearDisplay();
	_stat_draw_bounding_boxes();
	StatsBox[0].draw_all();
	StatsBox[1].draw_all();
	StatsBox[2].draw_all();
	StatsBox[3].draw_all();
	StatsBox[3].draw_glyph();
	StatsBox[1].print_wrapped("AAAAA!!! Napierdalać!!!");
#endif
}

#endif
