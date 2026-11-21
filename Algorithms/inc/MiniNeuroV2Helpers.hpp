/*
 * MiniNeuroV2Helpers.hpp
 *
 * miniNeuroV2: fork of MiniNeuroV1Helpers.hpp (itself V4's helpers with run-time parameters)
 * plus the network side: the rear sensor's SigPerSPAD gate, the ToF history and detection
 * memory the network reads, the network step (state NEURAL) and the teacher recording hook.
 *
 * The heading-PID relay auto-tuner, the sensing/FSM helpers and the
 * movement sub-routines of the strategy.
 *
 * This is an *internal* header shared only by the ROBOT_MINI_NEURO_V2 translation units
 * (miniNeuroV2.cpp + MiniNeuroV2Helpers.cpp): it declares, with external linkage, the
 * file-scope objects those two units share (display, logger, IMU, ...) whose single
 * definitions live in miniNeuroV2.cpp. Include it only from a source guarded by
 * #ifdef ROBOT_MINI_NEURO_V2.
 */

#ifndef ALGORITHMS_INC_MININEUROV2HELPERS_HPP_
#define ALGORITHMS_INC_MININEUROV2HELPERS_HPP_

#include "MiniNeuroV2Params.hpp"
#include "MiniNeuroV2Net.hpp"

#include "cmsis_os2.h"
#include "osapi.h"
#include "osapi_mortal_thread.h"
#include "semphr.h"
#include "Adafruit_SSD1327.h"
#include "DirtyLogger.hpp"
#include "lsm6dso.hpp"
#include "vl53l1x.hpp"
#include "L9960T.hpp"
#include "HeadingPID.hpp"
#include "Algo.hpp"
#include "RobotHardware.hpp"   // IMU, Sensors, MOTOR_CONTROLLERS, logger, HAL callbacks; display, DisplayThread, StatsBox (DEBUG_SCREEN)
#include "Manoeuvre.hpp"
#include <cstring>

// --- Tuning shared between the strategy file and its helpers: V4's constants, now read
// from Neuro2Params (defaults = V4 values). FULL_SPEED / MANOUVER_SPEED are gone: every use
// has its own parameter (attack_speed, turn_*_speed, search_*_speed, oob_*_speed).
#define DETECTION_DISTANCE (Neuro2Params.detection_distance_mm)
#define ROTATION_TIMEOUT ((uint32_t)Neuro2Params.rotation_timeout_ms)
#define MAX_SIG_PER_SPAD ((uint32_t)Neuro2Params.sig_threshold_cap)
// Calibrate() collects SigPerSPAD for this long (V4 used ROTATION_TIMEOUT, also 150 ms).
#define NEURO_CALIBRATION_MS 150

// Robot::loop() wake-up bits in the ToF event group (bit n = Sensors[n] sample ready).
// LINE_EVENT_BIT is set by PeriodicCheckCall() on a new line detection.
#define LINE_EVENT_BIT   ((EventBits_t)(1u << 20))
#define TOF_FRONT_MASK   ((EventBits_t)((1u << 0) | (1u << 2)))   // Sensors[0] and Sensors[2]
// Opponent is re-evaluated at least this often, even if a front sensor stops reporting.
#define TOF_EVAL_TIMEOUT_MS  ((uint32_t)Neuro2Params.tof_eval_timeout_ms)
// A sensor without a sample for this long (~2 periods) counts as "no detection".
#define TOF_STALE_MS         ((int32_t)Neuro2Params.tof_stale_ms)
// isOpponentDetected() TRACE records (ToF + orientation) at most this often: it now runs
// on every front sample (~2 per period), and the 8 KB log ring must survive SD stalls.
#define TOF_TRACE_LOG_PERIOD_MS  20

// Errors smaller than this are treated as zero, so sensor noise / motor response lag
// doesn't make the robot hunt back and forth for a negligible heading difference.
#define HEADING_PID_DEADBAND_DEG 15.0f

namespace neuro_v2 {
// Match FSM states: the strategy file (miniNeuroV2.cpp) drives the machine, while
// LookForOpponent2() returns the next state to enter. NEURAL: the network drives (policy_mode 1);
// it replaces FIGHT, LOOK_FOR_OPPONENT_1 and LOOK_FOR_OPPONENT_2.
typedef enum{CALIBRATE, WAIT_FOR_START, FIGHT, OUT_OF_BOUNDS, LOOK_FOR_OPPONENT_1, LOOK_FOR_OPPONENT_2, NEURAL, NOP} State_t;
}

// The strategy (Robot, Algo.hpp); CreateMiniNeuroV2() in miniNeuroV2.cpp creates it.
class MiniNeuroV2 : public Robot
{
public:
	void PeriodCB(void) override;
	void PeriodicCheckCall(void) override;
	bool isOutOfBounds(void) override;
protected:
	void Calibrate(void) override;
	SensorDistances_t GetValidatedDistances(void) override;
	void begin(void) override;
	void loop(void) override;
	void end(void) override;
private:
	void FightAlgorithm(void);
	neuro_v2::State_t CheckStartCondition(neuro_v2::State_t eFSM_state);
	bool isOpponentDetected(void);   // non-blocking, evaluates the latest samples
	bool OobTurnLeft(void) const;
};

/*
 * State and sub-routines of this strategy, in its own namespace (one firmware may hold several
 * strategies). lastDetOnLeft is defined in miniNeuroV2.cpp, the functions in MiniNeuroV2Helpers.cpp.
 */
namespace neuro_v2 {
// Side the opponent was last seen on: steers the search and the out-of-bounds escape.
extern volatile bool lastDetOnLeft;
// State name for the logs ("FSM: A -> B"; SumoSimulation reads the state after the arrow).
const char* StateName(State_t state);
// Heading-PID relay auto-tune (not called in a match; see its definition).
void PID_CalibrateHeading(HeadingPID& pid, uint32_t maxDurationMs, float relayPower);
// Search manoeuvre of LOOK_FOR_OPPONENT_2; returns the next state.
State_t LookForOpponent2(Robot *obj, Manoeuvre *m, bool opponent_detected);
}

/*
 * Network side (implemented in MiniNeuroV2Helpers.cpp).
 */
// Every motor command goes through here, so the network sees (and the teacher recording
// stores) the command actually given.
void Neuro2SetMotors(float left, float right);
// Updates the ToF history and the detection memory with new samples (the strategy's
// GetValidatedDistances()); call on every pass.
void Neuro2UpdateMemory(const SensorDistances_t& d);
// The match started (start signal): resets the memory and the "time since start" input, and
// takes the search direction chosen before the start (lastDetOnLeft, direction_ack).
void Neuro2MatchStarted(void);
// A line sensor sees the edge (called by MiniNeuroV2::isOutOfBounds()): the line memory inputs.
void Neuro2LineSeen(bool left, bool right);
// Builds the network inputs from the history, the memory, the IMU and the last command.
void Neuro2Features(float in[NEURO2_INPUTS]);
// One NEURAL pass: inputs -> network -> motors. Also keeps lastDetOnLeft (OOB escape side).
void Neuro2Step(Robot *obj);
// Teacher recording (simulator only): `in` built before the V1 FSM decided, and the command
// it gave. No-op on the robot.
void Neuro2RecordTeacher(const float in[NEURO2_INPUTS]);

#endif /* ALGORITHMS_INC_MININEUROV2HELPERS_HPP_ */
