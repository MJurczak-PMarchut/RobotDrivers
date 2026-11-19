/*
 * MiniNeuroV1Params.hpp
 *
 * Run-time parameters of the miniNeuroV1 strategy. miniNeuroV1 is the MiniTomi V4 match FSM
 * with its tuning constants turned into these parameters, so that SumoSimulation can tune
 * them (tools/optimize_neuro.py). The defaults are exactly the V4 values: with them
 * miniNeuroV1 behaves like miniTomiV4.
 *
 * All values are doubles so the default arithmetic matches V4's #define constants bit for
 * bit; integer-valued parameters (ms, mm, modes) are rounded when set.
 */

#ifndef ALGORITHMS_INC_MININEUROV1PARAMS_HPP_
#define ALGORITHMS_INC_MININEUROV1PARAMS_HPP_

#include <stddef.h>

typedef struct {
	// Opponent detection
	double detection_distance_mm;     // DETECTION_DISTANCE: opponent if a validated range is below
	double forward_band_mm;           // FightAlgorithm: |L - R| below this -> straight attack
	double tof_eval_timeout_ms;       // TOF_EVAL_TIMEOUT_MS
	double tof_stale_ms;              // TOF_STALE_MS
	double sig_threshold_gain;        // Calibrate(): SigPerSPAD threshold = gain * max seen
	double sig_threshold_cap;         // MAX_SIG_PER_SPAD: upper limit of that threshold

	// Motor powers [-1, 1]
	double attack_speed;              // ATTACK FWD (V4: FULL_SPEED)
	double turn_outer_speed;          // TURN LEFT/RIGHT, outer wheel (V4: FULL_SPEED)
	double turn_inner_speed;          // TURN LEFT/RIGHT, inner wheel (V4: MANOUVER_SPEED)
	double search_spin_speed;         // LOOK_FOR_OPPONENT_1 spin (V4: MANOUVER_SPEED)
	double search2_speed;             // LOOK_FOR_OPPONENT_2 drive, back off and turn (V4: MANOUVER_SPEED)
	double oob_back_speed;            // OUT_OF_BOUNDS back off (V4: MANOUVER_SPEED)
	double oob_turn_speed;            // OUT_OF_BOUNDS turn (V4: MANOUVER_SPEED)

	// Timings
	double oob_stop_ms;               // OOB_STOP_MS
	double oob_back_ms;               // OOB_BACK_MS
	double oob_turn_ms;               // OOB_TURN_MS
	double search_strategy_change_ms; // SEARCH_STRATEGY_CHANGE_TIME: LOOK_1 -> LOOK_2
	double search2_back_ms;           // SEARCH2_BACK_MS
	double search2_turn_deg;          // SEARCH2_TURN_DEG
	double rotation_timeout_ms;       // ROTATION_TIMEOUT (search 2 turn limit)

	// Heading PID (output limit and antipode hysteresis stay as in V4)
	double pid_kp;
	double pid_ki;
	double pid_kd;

	// Behaviour switches
	double oob_turn_mode;             // 0: turn towards the last detection (V4)
	                                  // 1: turn away from the line sensor that fired (if only one did)
} MiniNeuroV1Params_t;

extern MiniNeuroV1Params_t NeuroParams;

typedef struct {
	const char *name;
	double *value;
	double min;
	double max;
	double default_value;
	bool integer;
} MiniNeuroV1ParamInfo_t;

size_t MiniNeuroV1_ParamCount(void);
const MiniNeuroV1ParamInfo_t *MiniNeuroV1_ParamInfo(size_t index);
// Sets a parameter by name, clamped to its range (and rounded if integer). False if unknown.
bool MiniNeuroV1_SetParam(const char *name, double value);
void MiniNeuroV1_ResetParams(void);

#endif /* ALGORITHMS_INC_MININEUROV1PARAMS_HPP_ */
