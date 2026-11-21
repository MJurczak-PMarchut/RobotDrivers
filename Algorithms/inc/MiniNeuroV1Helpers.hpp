/*
 * MiniNeuroV1Helpers.hpp
 *
 * miniNeuroV1: copy of MiniTomiV4Helpers.hpp (RobotDrivers 7db6ebe) whose V4 tuning
 * constants are run-time parameters (MiniNeuroV1Params.hpp), tuned in SumoSimulation.
 *
 * The heading-PID relay auto-tuner, the sensing/FSM helpers and the
 * movement sub-routines of the strategy.
 *
 * This is an *internal* header shared only by the ROBOT_MINI_NEURO_V1 translation units
 * (miniNeuroV1.cpp + MiniNeuroV1Helpers.cpp): it declares, with external linkage, the
 * file-scope objects those two units share (display, logger, IMU, ...) whose single
 * definitions live in miniNeuroV1.cpp. Include it only from a source guarded by
 * #ifdef ROBOT_MINI_NEURO_V1.
 */

#ifndef ALGORITHMS_INC_MININEUROV1HELPERS_HPP_
#define ALGORITHMS_INC_MININEUROV1HELPERS_HPP_

#include "MiniNeuroV1Params.hpp"

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
// from NeuroParams (defaults = V4 values). FULL_SPEED / MANOUVER_SPEED are gone: every use
// has its own parameter (attack_speed, turn_*_speed, search_*_speed, oob_*_speed).
#define DETECTION_DISTANCE (NeuroParams.detection_distance_mm)
#define ROTATION_TIMEOUT ((uint32_t)NeuroParams.rotation_timeout_ms)
#define MAX_SIG_PER_SPAD ((uint32_t)NeuroParams.sig_threshold_cap)
// Calibrate() collects SigPerSPAD for this long (V4 used ROTATION_TIMEOUT, also 150 ms).
#define NEURO_CALIBRATION_MS 150

// Robot::loop() wake-up bits in the ToF event group (bit n = Sensors[n] sample ready).
// LINE_EVENT_BIT is set by PeriodicCheckCall() on a new line detection.
#define LINE_EVENT_BIT   ((EventBits_t)(1u << 20))
#define TOF_FRONT_MASK   ((EventBits_t)((1u << 0) | (1u << 2)))   // Sensors[0] and Sensors[2]
// Opponent is re-evaluated at least this often, even if a front sensor stops reporting.
#define TOF_EVAL_TIMEOUT_MS  ((uint32_t)NeuroParams.tof_eval_timeout_ms)
// A sensor without a sample for this long (~2 periods) counts as "no detection".
#define TOF_STALE_MS         ((int32_t)NeuroParams.tof_stale_ms)
// isOpponentDetected() TRACE records (ToF + orientation) at most this often: it now runs
// on every front sample (~2 per period), and the 8 KB log ring must survive SD stalls.
#define TOF_TRACE_LOG_PERIOD_MS  20

// Errors smaller than this are treated as zero, so sensor noise / motor response lag
// doesn't make the robot hunt back and forth for a negligible heading difference.
#define HEADING_PID_DEADBAND_DEG 15.0f

namespace neuro_v1 {
// Match FSM states: the strategy file (miniNeuroV1.cpp) drives the machine, while
// LookForOpponent2() returns the next state to enter.
typedef enum{CALIBRATE, WAIT_FOR_START, FIGHT, OUT_OF_BOUNDS, LOOK_FOR_OPPONENT_1, LOOK_FOR_OPPONENT_2, NOP} State_t;
}

// The strategy (Robot, Algo.hpp); CreateMiniNeuroV1() in miniNeuroV1.cpp creates it.
class MiniNeuroV1 : public Robot
{
public:
	void PeriodCB(void) override;
	void PeriodicCheckCall(void) override;
protected:
	void Calibrate(void) override;
	SensorDistances_t GetValidatedDistances(void) override;
	void begin(void) override;
	void loop(void) override;
	void end(void) override;
private:
	void FightAlgorithm(void);
	neuro_v1::State_t CheckStartCondition(neuro_v1::State_t eFSM_state);
	bool isOpponentDetected(void);   // non-blocking, evaluates the latest samples
	bool OobTurnLeft(void) const;
};

/*
 * State and sub-routines of this strategy, in its own namespace (one firmware may hold several
 * strategies). lastDetOnLeft is defined in miniNeuroV1.cpp, the functions in MiniNeuroV1Helpers.cpp.
 */
namespace neuro_v1 {
// Side the opponent was last seen on: steers the search and the out-of-bounds escape.
extern volatile bool lastDetOnLeft;
// State name for the logs ("FSM: A -> B"; SumoSimulation reads the state after the arrow).
const char* StateName(State_t state);
// Heading-PID relay auto-tune (not called in a match; see its definition).
void PID_CalibrateHeading(HeadingPID& pid, uint32_t maxDurationMs, float relayPower);
// Search manoeuvre of LOOK_FOR_OPPONENT_2; returns the next state.
State_t LookForOpponent2(Robot *obj, Manoeuvre *m, bool opponent_detected);
}

#endif /* ALGORITHMS_INC_MININEUROV1HELPERS_HPP_ */
