/*
 * MiniTomiV4Helpers.cpp
 *
 * Implementations of the heading-PID relay auto-tuner, sensing/FSM helpers and movement
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
using namespace mt_v4;   // State_t, StateName, lastDetOnLeft, ... (MiniTomiV4Helpers.hpp)

// Latest per-sensor "opponent within DETECTION_DISTANCE" flags; only isOpponentDetected()
// reads/writes them, so they stay private to this TU.
static bool sensor_detected_item[2] = {false, false};

bool MiniTomiV4::isOpponentDetected(void)
{
	char sensor_message[128] = {0};
	static bool was_previously_detected= false;
	static bool was_stale[2] = {false, false};

	SensorDistances_t distances = GetValidatedDistances();
	if(distances.left.stale != was_stale[0] || distances.right.stale != was_stale[1])
	{
		sprintf(sensor_message, "ToF front stale: L=%u R=%u", distances.left.stale, distances.right.stale);
		logger.Log(sensor_message, LOGLEVEL_INFO);
		was_stale[0] = distances.left.stale;
		was_stale[1] = distances.right.stale;
	}
	// Rate-limited TRACE (TOF_TRACE_LOG_PERIOD_MS); INFO transitions below are not limited.
	static uint32_t last_trace_tick = 0;
	uint32_t now = HAL_GetTick();
	bool log_trace = (now - last_trace_tick) >= TOF_TRACE_LOG_PERIOD_MS;
	if(log_trace)
	{
		last_trace_tick = now;
		// [n] raw range, f[n] the range after the SigPerSPAD gate, s[n] the SigPerSPAD
		// that drove that gate. Raw stays first and keeps its original "[n]=" spelling so
		// logs from before the f/s fields existed still parse the same way. Worst case
		// (all six values 65535) is 76 chars + NUL, so the 128-byte buffer cannot overrun.
		sprintf(sensor_message, "ToF:\n\r[0]=%u\n\r[1]=%u\n\rf[0]=%u\n\rf[1]=%u\n\rs[0]=%u\n\rs[1]=%u\n\r",
				distances.left.raw, distances.right.raw,
				distances.left.filtered, distances.right.filtered,
				distances.left.sig, distances.right.sig);
		logger.Log(sensor_message,LOGLEVEL_TRACE);
	}
#ifdef DEBUG_SCREEN
	sprintf(sensor_message, "L=%u  ", distances.left.filtered);
	StatsBox[0].printstat(sensor_message, "L");


	sprintf(sensor_message, "P=%u  ", distances.right.filtered);
	StatsBox[0].printstat(sensor_message, "P");

	sprintf(sensor_message, "T=%u  ", distances.rear.raw);
	StatsBox[0].printstat(sensor_message, "T");
#endif
	sensor_detected_item[0] = distances.left.filtered < DETECTION_DISTANCE;
	sensor_detected_item[1] = distances.right.filtered < DETECTION_DISTANCE;
	bool detected = sensor_detected_item[0] || sensor_detected_item[1];

	if(was_previously_detected != detected){
		logger.Log((detected)? "Opponent Detected" : "No Oponnent", LOGLEVEL_INFO);
	}

	if(log_trace)
	{
		sprintf(sensor_message, "Orientation:\n\rx=%.2f\n\ry=%.2f\n\rz=%.2f\n\r", IMU.GetAngularOrientationForAxis(0), IMU.GetAngularOrientationForAxis(1), IMU.GetAngularOrientationForAxis(2));
		logger.Log(sensor_message, LOGLEVEL_TRACE);
	}

	was_previously_detected = detected;
	return detected;
}

const char* mt_v4::StateName(State_t state)
{
	static const char *names[] = {"CALIBRATE", "WAIT_FOR_START", "FIGHT", "OUT_OF_BOUNDS", "LOOK_FOR_OPPONENT_1", "LOOK_FOR_OPPONENT_2"};
	// One name per state before NOP: a state added to State_t without a name does not compile.
	static_assert(std::size(names) == static_cast<std::size_t>(NOP), "StateName: one name per state before NOP");
	return (static_cast<std::size_t>(state) < std::size(names))? names[state] : "UNKNOWN";
}

State_t MiniTomiV4::CheckStartCondition(State_t eFSM_state)
{
	static bool start_condition_prev = false;
	State_t retval = eFSM_state;
#ifdef DEBUG_SCREEN
	static bool was_enabled = false;
#endif
	if(eFSM_state ==  CALIBRATE){
		return eFSM_state;
	}
	bool start_condition = HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin);
	if(start_condition != start_condition_prev)
	{
		logger.Log((start_condition)? " Start Condition Set" : " Start Condition Reset ", LOGLEVEL_INFO);
		if(!start_condition)//dummy
		{
#ifdef DEBUG_SCREEN
			if(was_enabled){
				display.setCursor(42,0);
				display.println("State: Disabled");
			}
#endif
			this->SetFlashPeriodMS(1000);
			MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
			MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
			retval= WAIT_FOR_START;
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
			retval=  (eFSM_state == WAIT_FOR_START)? FIGHT : eFSM_state;
		}
	}
	
	start_condition_prev = start_condition;
	return retval;
}



namespace mt_v4 {

// Relay (Astrom-Hagglund) auto-tune for the heading PID: drives the motors in a
// bang-bang pattern around the current heading to force a sustained oscillation,
// measures its period/amplitude, and derives Kp/Ki/Kd via the standard Ziegler-Nichols
// relay formulas. Runs for at most maxDurationMs, so it always returns in finite time.
// The robot WILL spin in place while this runs - only call it with the robot on a stand
// or clear of obstacles, never during a match.
void PID_CalibrateHeading(HeadingPID& pid, uint32_t maxDurationMs, float relayPower)
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
	pid.SetGains(tunedKp, tunedKi, tunedKd);
	char string[100] = {0};
	sprintf(string, "\n\rP= %f\n\rI= %f\n\rD= %f\n\r", tunedKp, tunedKi, tunedKd);
	logger.Log(string, LOGLEVEL_TRACE);
}

} // namespace mt_v4

void MiniTomiV4::FightAlgorithm(void)
{
	static bool forward_det;
	SensorDistances_t distances = GetValidatedDistances();
	uint16_t sensor_left = distances.left.filtered;
	uint16_t sensor_right = distances.right.filtered;
	// detected
	this->SetFlashPeriodMS(100);
	bool fwd_det_tmp = (abs(sensor_left - sensor_right) < 60)? true:false;
	// Latch the target heading at the moment the forward-attack decision flips, then hold
	// it for the whole charge so the PID (run by PeriodicCheckCall) steers the robot
	// straight at where it saw the opponent.
	if(fwd_det_tmp != forward_det)
	{
		this->SetTargetHeading((float)IMU.GetAngularOrientationForAxis(2));
	}
	forward_det = fwd_det_tmp;
	const char *fight_move;
	if (forward_det){
		// Positive error (target > current yaw) needs a CCW turn: left slower, right faster
		// (same convention the relay autotuner and LookForOpponent2 use).
		float heading_correction = this->GetHeadingCorrection(1-FULL_SPEED);
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

// Search 2: drive forward; on the edge back off, then turn towards the last detection
// until SEARCH2_TURN_DEG or ROTATION_TIMEOUT. Opponent is ignored while backing off.
#define SEARCH2_BACK_MS   350
#define SEARCH2_TURN_DEG  60.0f
enum {SEARCH2_BACK = MANOEUVRE_IDLE + 1, SEARCH2_TURN};

namespace mt_v4 {

State_t LookForOpponent2(Robot *obj, Manoeuvre *m, bool opponent_detected)
{
	obj->SetFlashPeriodMS(500);
	switch(m->phase)
	{
		case MANOEUVRE_IDLE:
			if(opponent_detected)
			{
				return FIGHT;
			}
			if(obj->isOutOfBounds())
			{
				char search_message[100];
				sprintf(search_message, "Search2: edge hit, backing off, rotate %s", (lastDetOnLeft)? "LEFT" : "RIGHT");
				logger.Log(search_message, LOGLEVEL_INFO);
				MCInterface::SetMotorsPower(-MANOUVER_SPEED, -MANOUVER_SPEED);
				m->EnterPhase(SEARCH2_BACK, SEARCH2_BACK_MS);
			}
			else
			{
				MCInterface::SetMotorsPower(MANOUVER_SPEED, MANOUVER_SPEED);
			}
			break;
		case SEARCH2_BACK:
			if(m->PhaseDone())
			{
				m->start_heading = (float)IMU.GetAngularOrientationForAxis(2);
				if(lastDetOnLeft)
					MCInterface::SetMotorsPower(-MANOUVER_SPEED, MANOUVER_SPEED);
				else
					MCInterface::SetMotorsPower(MANOUVER_SPEED, -MANOUVER_SPEED);
				m->EnterPhase(SEARCH2_TURN, ROTATION_TIMEOUT);
			}
			break;
		case SEARCH2_TURN:
		{
			if(opponent_detected)
			{
				m->End();
				if(lastDetOnLeft)
					MCInterface::SetMotorsPower(MANOUVER_SPEED, -MANOUVER_SPEED);
				else
					MCInterface::SetMotorsPower(-MANOUVER_SPEED, MANOUVER_SPEED);

				return FIGHT;
			}
			float turned = fabsf(HeadingPID::WrapAngleDeg180((float)IMU.GetAngularOrientationForAxis(2) - m->start_heading));
			if(turned >= SEARCH2_TURN_DEG)
			{
				MCInterface::SetMotorsPower(MANOUVER_SPEED, MANOUVER_SPEED);
				m->End();
			}
			else if(m->PhaseDone())
			{
				m->End();   // next step drives forward
			}
		}
		break;
		default:
			m->End();
			break;
	}
	return LOOK_FOR_OPPONENT_2;
}

} // namespace mt_v4


#endif
