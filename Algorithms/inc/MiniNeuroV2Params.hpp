/*
 * MiniNeuroV2Params.hpp
 *
 * Run-time parameters of the miniNeuroV2 strategy, set by SumoSimulation (tools/) and
 * compiled in as defaults for the robot:
 *  - the miniNeuroV1 parameters, which drive the FSM parts (signal gate, line escape) and, in
 *    policy_mode 0, the V1 fighting and searching (the teacher the network imitates first);
 *  - the network switches (policy_mode, net_history, net_use_rear);
 *  - the network weights nn_w000 .. (MiniNeuroV2Net.hpp), stored as floats.
 *
 * The V1 fields are doubles so their defaults reproduce V1 / V4 arithmetic exactly;
 * integer-valued parameters (ms, mm, modes) are rounded when set.
 */

#ifndef ALGORITHMS_INC_MININEUROV2PARAMS_HPP_
#define ALGORITHMS_INC_MININEUROV2PARAMS_HPP_

#include <stddef.h>

#include "MiniNeuroV2Net.hpp"

typedef struct {
	// Opponent detection
	double detection_distance_mm;     // opponent if a validated range is below (also: the network's "detection" memory)
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

	// Network (MiniNeuroV2Net.hpp)
	double policy_mode;               // 0: the V1 FSM fights and searches (teacher); 1: the network (state NEURAL)
	double net_history;               // ToF samples per front sensor the network sees: 1..3 (older ones read 0)
	double net_use_rear;              // 1: rear sensor inputs; 0: they read 0
} MiniNeuroV2Params_t;

extern MiniNeuroV2Params_t Neuro2Params;

typedef struct {
	const char *name;
	double *value;          // a field of Neuro2Params, or NULL for a network weight
	float *weight;          // an element of Neuro2Weights, or NULL
	double min;
	double max;
	double default_value;
	bool integer;
} MiniNeuroV2ParamInfo_t;

// Parameters: the fields above, then the weights nn_w000 .. nn_wNNN.
size_t MiniNeuroV2_ParamCount(void);
// Valid until the next call (weights share one entry).
const MiniNeuroV2ParamInfo_t *MiniNeuroV2_ParamInfo(size_t index);
// Sets a parameter by name, clamped to its range (and rounded if integer). False if unknown.
bool MiniNeuroV2_SetParam(const char *name, double value);
// Defaults for everything, weights included.
void MiniNeuroV2_ResetParams(void);

#endif /* ALGORITHMS_INC_MININEUROV2PARAMS_HPP_ */
