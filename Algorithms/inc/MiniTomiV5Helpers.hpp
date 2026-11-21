/*
 * MiniTomiV5Helpers.hpp
 *
 * miniTomiV5: copy of MiniTomiV4Helpers.hpp (RobotDrivers f937b21), compiled only for the ROBOT_MT_V5
 * build (MiniTomiV5 preset). Starts out identical to V4.
 *
 * The heading-PID relay auto-tuner, the sensing/FSM helpers and the
 * movement sub-routines that used to live inline in miniTomiV5.cpp. Splitting them
 * out keeps the V5 strategy file focused on Robot::begin/loop and the match FSM.
 *
 * This is an *internal* header shared only by the ROBOT_MT_V5 translation units
 * (miniTomiV5.cpp + MiniTomiV5Helpers.cpp): it declares, with external linkage, the
 * file-scope objects those two units share (display, logger, IMU, ...) whose single
 * definitions live in miniTomiV5.cpp. Include it only from a source guarded by
 * #ifdef ROBOT_MT_V5.
 */

#ifndef ALGORITHMS_INC_MINITOMIV5HELPERS_HPP_
#define ALGORITHMS_INC_MINITOMIV5HELPERS_HPP_

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

// --- V5 tuning constants (V4 values) shared between the strategy file and its helpers ---
#define DETECTION_DISTANCE Robot::kDetectionDistanceMm   // the default GetValidatedDistances() (Algo.hpp)
#define FULL_SPEED 0.85
#define MANOUVER_SPEED 0.65
#define ROTATION_TIMEOUT 150
#define MAX_SIG_PER_SPAD	8000

// Robot::loop() wake-up bits in the ToF event group (bit n = Sensors[n] sample ready).
// LINE_EVENT_BIT is set by PeriodicCheckCall() on a new line detection.
#define LINE_EVENT_BIT   ((EventBits_t)(1u << 20))
#define TOF_FRONT_MASK   ((EventBits_t)((1u << 0) | (1u << 2)))   // Sensors[0] and Sensors[2]
// Opponent is re-evaluated at least this often, even if a front sensor stops reporting.
#define TOF_EVAL_TIMEOUT_MS  30
// isOpponentDetected() TRACE records (ToF + orientation) at most this often: it now runs
// on every front sample (~2 per period), and the 8 KB log ring must survive SD stalls.
#define TOF_TRACE_LOG_PERIOD_MS  20

// Errors smaller than this are treated as zero, so sensor noise / motor response lag
// doesn't make the robot hunt back and forth for a negligible heading difference.
#define HEADING_PID_DEADBAND_DEG 15.0f

namespace mt_v5 {
// Match FSM states: the strategy file (miniTomiV5.cpp) drives the machine, while
// LookForOpponent2() returns the next state to enter.
typedef enum{CALIBRATE, WAIT_FOR_START, FIGHT, OUT_OF_BOUNDS, LOOK_FOR_OPPONENT_1, LOOK_FOR_OPPONENT_2, NOP} State_t;
}

// The strategy (Robot, Algo.hpp); CreateMiniTomiV5() in miniTomiV5.cpp creates it.
class MiniTomiV5 : public Robot
{
public:
	void PeriodCB(void) override;
	void PeriodicCheckCall(void) override;
protected:
	void begin(void) override;
	void loop(void) override;
	void end(void) override;
private:
	void FightAlgorithm(void);
	mt_v5::State_t CheckStartCondition(mt_v5::State_t eFSM_state);
	bool isOpponentDetected(void);   // non-blocking, evaluates the latest samples
};

/*
 * State and sub-routines of this strategy, in its own namespace (one firmware may hold several
 * strategies). lastDetOnLeft is defined in miniTomiV5.cpp, the functions in MiniTomiV5Helpers.cpp.
 */
namespace mt_v5 {
// Side the opponent was last seen on: steers the search and the out-of-bounds escape.
extern volatile bool lastDetOnLeft;
// State name for the logs ("FSM: A -> B"; SumoSimulation reads the state after the arrow).
const char* StateName(State_t state);
// Heading-PID relay auto-tune (not called in a match; see its definition).
void PID_CalibrateHeading(HeadingPID& pid, uint32_t maxDurationMs, float relayPower);
// Search manoeuvre of LOOK_FOR_OPPONENT_2; returns the next state.
State_t LookForOpponent2(Robot *obj, Manoeuvre *m, bool opponent_detected);
}

#endif /* ALGORITHMS_INC_MINITOMIV5HELPERS_HPP_ */
