/*
 * MiniNeuroV2Helpers.cpp
 *
 * miniNeuroV2: fork of MiniNeuroV1Helpers.cpp (V4's helpers with the tuning constants read
 * from the parameters), plus the rear sensor's SigPerSPAD gate and the network side at the
 * end of the file. See MiniNeuroV2Helpers.hpp for the shared declarations. The whole unit is
 * compiled only for the ROBOT_MINI_NEURO_V2 build.
 */
#include "cmsis_os2.h"
#include "osapi.h"
#include "portmacro.h"
#include "stm32h7xx_hal.h"
#include <cmath>
#include <stdio.h>
#include <string.h>
#if defined(ROBOT_MINI_NEURO_V2) || defined(ROBOT_ALGO_SELECT)   // ROBOT_ALGO_SELECT: all strategies in one image
#include "MiniNeuroV2Helpers.hpp"
using namespace neuro_v2;   // State_t, StateName, lastDetOnLeft, ... (MiniNeuroV2Helpers.hpp)

// Latest per-sensor "opponent within DETECTION_DISTANCE" flags; only isOpponentDetected()
// reads/writes them, so they stay private to this TU.
static bool sensor_detected_item[2] = {false, false};

// Robot::isOutOfBounds() plus the network's line memory (no effect on the FSM).
bool MiniNeuroV2::isOutOfBounds(void)
{
	const bool out = Robot::isOutOfBounds();
	if(out)
	{
		Neuro2LineSeen(LineDetOnLeft(), LineDetOnRight());
	}
	return out;
}

// Robot::GetValidatedDistances() with the limits from Neuro2Params (DETECTION_DISTANCE,
// TOF_STALE_MS) instead of Robot's constants.
SensorDistances_t MiniNeuroV2::GetValidatedDistances(void)
{
	SensorDistances_t distances;
	ToFReading_t* readings[] = {&distances.left, &distances.rear, &distances.right};
	static_assert(std::size(readings) == std::size(Sensors));
	uint32_t now = HAL_GetTick();
	// Same order as Sensors[]: 0 = left, 1 = rear, 2 = right
	for(std::size_t sensor = 0; sensor < std::size(Sensors); sensor++){
		ToFReading_t& r = *readings[sensor];
		// Signed age: an ISR may store a tick newer than `now` between the two reads (B16).
		r.stale = (int32_t)(now - Sensors[sensor].GetLastSampleTick()) > TOF_STALE_MS;
		r.raw = Sensors[sensor].GetDistance();
		r.sig = Sensors[sensor].GetResult().SigPerSPAD;
		r.filtered = (!r.stale && r.sig > SigPerSPADThreshold[sensor])? r.raw: DETECTION_DISTANCE + 100;
	}
	return distances;
}

bool MiniNeuroV2::isOpponentDetected(void)
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
		// that drove that gate; [0]/[1] front left/right as in V1, [2] the rear, appended so
		// V1 log parsers still read the first six. Worst case (all nine values 65535) is
		// 114 chars + NUL, so the 128-byte buffer cannot overrun.
		sprintf(sensor_message, "ToF:\n\r[0]=%u\n\r[1]=%u\n\rf[0]=%u\n\rf[1]=%u\n\rs[0]=%u\n\rs[1]=%u\n\r"
				"[2]=%u\n\rf[2]=%u\n\rs[2]=%u\n\r",
				distances.left.raw, distances.right.raw,
				distances.left.filtered, distances.right.filtered,
				distances.left.sig, distances.right.sig,
				distances.rear.raw, distances.rear.filtered, distances.rear.sig);
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

const char* neuro_v2::StateName(State_t state)
{
	static const char *names[] = {"CALIBRATE", "WAIT_FOR_START", "FIGHT", "OUT_OF_BOUNDS", "LOOK_FOR_OPPONENT_1", "LOOK_FOR_OPPONENT_2", "NEURAL"};
	// One name per state before NOP: a state added to State_t without a name does not compile.
	static_assert(std::size(names) == static_cast<std::size_t>(NOP), "StateName: one name per state before NOP");
	return (static_cast<std::size_t>(state) < std::size(names))? names[state] : "UNKNOWN";
}

State_t MiniNeuroV2::CheckStartCondition(State_t eFSM_state)
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



namespace neuro_v2 {

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
			Neuro2SetMotors(-relayPower, relayPower);
		}
		else
		{
			Neuro2SetMotors(relayPower, -relayPower);
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	Neuro2SetMotors(0.0f, 0.0f);

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

} // namespace neuro_v2

void MiniNeuroV2::FightAlgorithm(void)
{
	static bool forward_det;
	SensorDistances_t distances = GetValidatedDistances();
	uint16_t sensor_left = distances.left.filtered;
	uint16_t sensor_right = distances.right.filtered;
	// detected
	this->SetFlashPeriodMS(100);
	bool fwd_det_tmp = (abs(sensor_left - sensor_right) < Neuro2Params.forward_band_mm)? true:false;
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
		float heading_correction = this->GetHeadingCorrection(1-Neuro2Params.attack_speed);
		Neuro2SetMotors(Neuro2Params.attack_speed - heading_correction, Neuro2Params.attack_speed + heading_correction);
		fight_move = "ATTACK FWD";
	}
	else if(sensor_left < sensor_right)
	{
		// on the left
		Neuro2SetMotors(Neuro2Params.turn_inner_speed, Neuro2Params.turn_outer_speed);
		lastDetOnLeft = true;
		fight_move = "TURN LEFT";
	}
	else{
		Neuro2SetMotors(Neuro2Params.turn_outer_speed, Neuro2Params.turn_inner_speed);
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
#define SEARCH2_BACK_MS   ((uint32_t)Neuro2Params.search2_back_ms)
#define SEARCH2_TURN_DEG  (Neuro2Params.search2_turn_deg)
enum {SEARCH2_BACK = MANOEUVRE_IDLE + 1, SEARCH2_TURN};

namespace neuro_v2 {

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
				Neuro2SetMotors(-Neuro2Params.search2_speed, -Neuro2Params.search2_speed);
				m->EnterPhase(SEARCH2_BACK, SEARCH2_BACK_MS);
			}
			else
			{
				Neuro2SetMotors(Neuro2Params.search2_speed, Neuro2Params.search2_speed);
			}
			break;
		case SEARCH2_BACK:
			if(m->PhaseDone())
			{
				m->start_heading = (float)IMU.GetAngularOrientationForAxis(2);
				if(lastDetOnLeft)
					Neuro2SetMotors(-Neuro2Params.search2_speed, Neuro2Params.search2_speed);
				else
					Neuro2SetMotors(Neuro2Params.search2_speed, -Neuro2Params.search2_speed);
				m->EnterPhase(SEARCH2_TURN, ROTATION_TIMEOUT);
			}
			break;
		case SEARCH2_TURN:
		{
			if(opponent_detected)
			{
				m->End();
				if(lastDetOnLeft)
					Neuro2SetMotors(Neuro2Params.search2_speed, -Neuro2Params.search2_speed);
				else
					Neuro2SetMotors(-Neuro2Params.search2_speed, Neuro2Params.search2_speed);

				return FIGHT;
			}
			float turned = fabsf(HeadingPID::WrapAngleDeg180((float)IMU.GetAngularOrientationForAxis(2) - m->start_heading));
			if(turned >= SEARCH2_TURN_DEG)
			{
				Neuro2SetMotors(Neuro2Params.search2_speed, Neuro2Params.search2_speed);
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

} // namespace neuro_v2

void MiniNeuroV2::Calibrate(void)
{
	TickType_t start_time = xTaskGetTickCount();
	while((xTaskGetTickCount() - start_time) < NEURO_CALIBRATION_MS){
		xEventGroupWaitBits(ToF_Sensor::GetEventHandle(), TOF_EVENT_MASK, pdTRUE, pdTRUE, 30);
		VL53L1X_Result_t result_left = Sensors[0].GetResult();
		VL53L1X_Result_t result_right = Sensors[2].GetResult();
		VL53L1X_Result_t result_rear = Sensors[1].GetResult();
		SigPerSPADThreshold[0] = (SigPerSPADThreshold[0] < result_left.SigPerSPAD)? result_left.SigPerSPAD: SigPerSPADThreshold[0];
		SigPerSPADThreshold[2] = (SigPerSPADThreshold[2] < result_right.SigPerSPAD)? result_right.SigPerSPAD: SigPerSPADThreshold[2];
		SigPerSPADThreshold[1] = (SigPerSPADThreshold[1] < result_rear.SigPerSPAD)? result_rear.SigPerSPAD: SigPerSPADThreshold[1];
	}
	// Threshold = gain * max seen, capped. Computed in 32 bits: V4 stores max*2 straight into
	// the uint16_t, which wraps (e.g. 33000 -> 464) when something strong is close at boot.
	for (std::size_t sensor = 0; sensor < std::size(SigPerSPADThreshold); sensor++)
	{
		uint32_t threshold = (uint32_t)lround(SigPerSPADThreshold[sensor] * Neuro2Params.sig_threshold_gain);
		SigPerSPADThreshold[sensor] = (uint16_t)((threshold < MAX_SIG_PER_SPAD) ? threshold : MAX_SIG_PER_SPAD);
	}

	char calib_message[100];
	sprintf(calib_message, "Calib SigPerSPAD thresholds L=%u R=%u B=%u", SigPerSPADThreshold[0], SigPerSPADThreshold[2], SigPerSPADThreshold[1]);
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
	StatsBox[2].draw_glyph(kGlyphMiniNeuroV2);   // which strategy runs (StrategyGlyphs.hpp)
	StatsBox[1].print_wrapped("AAAAA!!! Napierdalać!!!");
#endif
}

/*
 * Network side
 */

// Last command (Neuro2SetMotors): the network's previous-command inputs and the teacher label.
static float s_cmd_left = 0.0f;
static float s_cmd_right = 0.0f;

void Neuro2SetMotors(float left, float right)
{
	s_cmd_left = left;
	s_cmd_right = right;
	MCInterface::SetMotorsPower(left, right);
}

// ToF history, newest first, normalised range (0 = touching .. 1 = NEURO2_FAR_MM or more, also
// "no valid target"): slot 0 front left (Sensors[0]), 1 front right (Sensors[2]), 2 rear (Sensors[1]).
#define NEURO2_FAR_MM 1000.0f
#define NEURO2_HISTORY 3
static const uint8_t kSlotSensor[3] = {0, 2, 1};
static float s_hist[3][NEURO2_HISTORY] = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
static uint32_t s_sample_tick[3] = {0, 0, 0};
static bool s_have_sample[3] = {false, false, false};
static bool s_stale[3] = {true, true, true};

// Detection memory: last front detection (a gated range below detection_distance_mm).
static bool s_det_valid = false;
static TickType_t s_det_tick = 0;
static float s_det_side = 0.0f;      // +1 left, -1 right
static float s_det_heading = 0.0f;   // yaw then [deg]

static bool s_started = false;
static TickType_t s_start_tick = 0;
static float s_start_dir = -1.0f;    // lastDetOnLeft at the start: +1 left, -1 right (default)

// Line memory: the last time a line sensor saw the white edge (isOutOfBounds()).
static bool s_line_valid = false;
static TickType_t s_line_tick = 0;
static float s_line_side = 0.0f;     // +1 left, -1 right, 0 both
static float s_line_heading = 0.0f;  // yaw then [deg]

// Forward attack, as V1's FightAlgorithm decides it (|left - right| < forward_band_mm on the
// validated distances): the heading when that decision last flipped, which V1 latches as the
// heading PID's target.
static bool s_fwd_known = false;
static bool s_fwd = false;
static float s_fwd_heading = 0.0f;

static float GatedRange(uint16_t raw, uint16_t sig, uint16_t threshold, bool stale)
{
	if (stale || sig <= threshold)
	{
		return 1.0f;
	}
	const float d = (float)raw / NEURO2_FAR_MM;
	return (d < 1.0f)? d : 1.0f;
}

static float Clip1(float x)
{
	return (x > 1.0f)? 1.0f : (x < -1.0f)? -1.0f : x;
}

void Neuro2UpdateMemory(const SensorDistances_t& d)
{
	const float range[3] = {
		GatedRange(d.left.raw, d.left.sig, SigPerSPADThreshold[0], d.left.stale),
		GatedRange(d.right.raw, d.right.sig, SigPerSPADThreshold[2], d.right.stale),
		GatedRange(d.rear.raw, d.rear.sig, SigPerSPADThreshold[1], d.rear.stale)};
	s_stale[0] = d.left.stale;
	s_stale[1] = d.right.stale;
	s_stale[2] = d.rear.stale;
	const bool fwd = abs((int)d.left.filtered - (int)d.right.filtered) < (int)Neuro2Params.forward_band_mm;
	if (!s_fwd_known || fwd != s_fwd)
	{
		s_fwd_heading = (float)IMU.GetAngularOrientationForAxis(2);
	}
	s_fwd_known = true;
	s_fwd = fwd;
	bool new_front = false;
	for (int slot = 0; slot < 3; slot++)
	{
		const uint32_t tick = Sensors[kSlotSensor[slot]].GetLastSampleTick();
		if (!s_have_sample[slot] || tick != s_sample_tick[slot])
		{
			s_have_sample[slot] = true;
			s_sample_tick[slot] = tick;
			for (int k = NEURO2_HISTORY - 1; k > 0; k--)
			{
				s_hist[slot][k] = s_hist[slot][k - 1];
			}
			s_hist[slot][0] = range[slot];
			new_front = new_front || (slot < 2);
		}
	}
	if (new_front)
	{
		const float limit = (float)(Neuro2Params.detection_distance_mm / NEURO2_FAR_MM);
		const bool left = !s_stale[0] && s_hist[0][0] < limit;
		const bool right = !s_stale[1] && s_hist[1][0] < limit;
		if (left || right)
		{
			s_det_valid = true;
			s_det_tick = xTaskGetTickCount();
			s_det_side = (left && right)? ((s_hist[0][0] <= s_hist[1][0])? 1.0f : -1.0f) : (left? 1.0f : -1.0f);
			s_det_heading = (float)IMU.GetAngularOrientationForAxis(2);
		}
	}
}

void Neuro2MatchStarted(void)
{
	s_started = true;
	s_start_tick = xTaskGetTickCount();
	s_det_valid = false;
	s_line_valid = false;
	// WAIT_FOR_START sets lastDetOnLeft when the operator turns the robot (direction_ack);
	// V1 searches towards it, so the network needs it too.
	s_start_dir = lastDetOnLeft? 1.0f : -1.0f;
}

void Neuro2LineSeen(bool left, bool right)
{
	s_line_valid = true;
	s_line_tick = xTaskGetTickCount();
	s_line_side = (left && right)? 0.0f : (left? 1.0f : -1.0f);
	s_line_heading = (float)IMU.GetAngularOrientationForAxis(2);
}

void Neuro2Features(float in[NEURO2_INPUTS])
{
	const TickType_t now = xTaskGetTickCount();
	const int history = (int)Neuro2Params.net_history;
	for (int slot = 0; slot < 2; slot++)
	{
		for (int k = 0; k < NEURO2_HISTORY; k++)
		{
			const float v = s_stale[slot]? 1.0f : s_hist[slot][k];
			in[NN_IN_FL0 + slot * NEURO2_HISTORY + k] = (k < history)? v : 0.0f;
		}
	}
	const bool rear = Neuro2Params.net_use_rear >= 0.5;
	in[NN_IN_RR0] = rear? (s_stale[2]? 1.0f : s_hist[2][0]) : 0.0f;
	in[NN_IN_RR1] = rear? (s_stale[2]? 1.0f : s_hist[2][1]) : 0.0f;

	const float heading = (float)IMU.GetAngularOrientationForAxis(2);
	if (s_det_valid)
	{
		const float since = (float)(now - s_det_tick) / 2000.0f;
		const float since_long = (float)(now - s_det_tick) / 10000.0f;
		in[NN_IN_SINCE_DET] = (since < 1.0f)? since : 1.0f;
		in[NN_IN_SINCE_DET_LONG] = (since_long < 1.0f)? since_long : 1.0f;
		in[NN_IN_DET_SIDE] = s_det_side;
		in[NN_IN_TURNED] = HeadingPID::WrapAngleDeg180(heading - s_det_heading) / 180.0f;
	}
	else
	{
		in[NN_IN_SINCE_DET] = 1.0f;
		in[NN_IN_SINCE_DET_LONG] = 1.0f;
		in[NN_IN_DET_SIDE] = 0.0f;
		in[NN_IN_TURNED] = 0.0f;
	}
	if (s_line_valid)
	{
		const float since = (float)(now - s_line_tick) / 2000.0f;
		in[NN_IN_SINCE_LINE] = (since < 1.0f)? since : 1.0f;
		in[NN_IN_LINE_SIDE] = s_line_side;
		in[NN_IN_LINE_TURNED] = HeadingPID::WrapAngleDeg180(heading - s_line_heading) / 180.0f;
	}
	else
	{
		in[NN_IN_SINCE_LINE] = 1.0f;
		in[NN_IN_LINE_SIDE] = 0.0f;
		in[NN_IN_LINE_TURNED] = 0.0f;
	}
	// Same sign as V1's heading error: positive (target counter-clockwise of us) needs a CCW turn.
	in[NN_IN_ATTACK_ERR] = s_fwd? HeadingPID::WrapAngleDeg180(s_fwd_heading - heading) / 180.0f : 0.0f;
	in[NN_IN_START_DIR] = s_start_dir;
	in[NN_IN_GYRO] = Clip1((float)IMU.GetAngularRateForAxis(2) / 720.0f);
	// Horizontal magnitude: does not depend on how the IMU is mounted.
	const float ax = (float)IMU.GetAccelerationForAxis(0);
	const float ay = (float)IMU.GetAccelerationForAxis(1);
	in[NN_IN_ACCEL_X] = Clip1(sqrtf(ax * ax + ay * ay) / 2.0f);
	in[NN_IN_PREV_LEFT] = s_cmd_left;
	in[NN_IN_PREV_RIGHT] = s_cmd_right;
	if (s_started)
	{
		const float since = (float)(now - s_start_tick) / 3000.0f;
		in[NN_IN_SINCE_START] = (since < 1.0f)? since : 1.0f;
	}
	else
	{
		in[NN_IN_SINCE_START] = 0.0f;
	}
}

void Neuro2Step(Robot *obj)
{
	float in[NEURO2_INPUTS];
	float out[NEURO2_OUTPUTS];
	Neuro2Features(in);
	Neuro2Forward(in, out);
	Neuro2SetMotors(out[0], out[1]);
	if (s_det_valid)
	{
		lastDetOnLeft = s_det_side > 0.0f;   // the OOB escape turns towards it (oob_turn_mode 0)
	}
	obj->SetFlashPeriodMS(100);

	// Rate-limited TRACE of the network's decision.
	static uint32_t last_trace_tick = 0;
	const uint32_t now = HAL_GetTick();
	if ((now - last_trace_tick) >= 100)
	{
		last_trace_tick = now;
		char message[64];
		sprintf(message, "NN: L=%.2f R=%.2f", out[0], out[1]);
		logger.Log(message, LOGLEVEL_TRACE);
	}
}

// Simulator hook (SumoSimulation, behaviour cloning). Weak and not defined in the robot
// build, so its address is NULL there and nothing is recorded.
extern "C" void MiniNeuroV2_RecordSample(const float *in, int count, float left, float right) __attribute__((weak));

void Neuro2RecordTeacher(const float in[NEURO2_INPUTS])
{
	if (MiniNeuroV2_RecordSample)
	{
		MiniNeuroV2_RecordSample(in, NEURO2_INPUTS, s_cmd_left, s_cmd_right);
	}
}

#endif
