/*
 * MiniNeuroV1Params.cpp
 *
 * Defaults and ranges of the miniNeuroV1 parameters (see MiniNeuroV1Params.hpp). The ranges
 * bound what the optimiser in SumoSimulation may try. Some lower limits are there because the
 * simulator is simpler than the robot, not because the strategy needs them:
 *  - tof_stale_ms >= 45: samples come every 20 ms with at most 1-3 ms jitter on a healthy
 *    sensor, so 45 ms tolerates one lost sample;
 *  - oob_back_ms >= 150 and oob_back_speed >= 0.4: braking at the edge is not calibrated, so
 *    the escape always backs off at least a few cm;
 *  - motor powers for search and escape >= 0.3: the simulated motors have no dead band
 *    (assumption, check that the robot really turns at 0.3);
 *  - pid_ki <= 0.15;
 *  - sig_threshold_gain >= 1.8: replaying the 150 ms calibration on the robot's ToF logs
 *    (Weryfikacja/TofDebug100), lower gains let no-target noise through as short ranges.
 */
#ifdef ROBOT_MINI_NEURO_V1
#include "MiniNeuroV1Params.hpp"
#include <math.h>
#include <string.h>

// Defaults = miniTomiV4 constants.
static const MiniNeuroV1Params_t kDefaults = {
	/* detection_distance_mm     */ 350,
	/* forward_band_mm           */ 60,
	/* tof_eval_timeout_ms       */ 30,
	/* tof_stale_ms              */ 60,
	/* sig_threshold_gain        */ 2.0,
	/* sig_threshold_cap         */ 8000,
	/* attack_speed              */ 0.85,
	/* turn_outer_speed          */ 0.85,
	/* turn_inner_speed          */ 0.65,
	/* search_spin_speed         */ 0.65,
	/* search2_speed             */ 0.65,
	/* oob_back_speed            */ 0.65,
	/* oob_turn_speed            */ 0.65,
	/* oob_stop_ms               */ 10,
	/* oob_back_ms               */ 200,
	/* oob_turn_ms               */ 100,
	/* search_strategy_change_ms */ 3000,
	/* search2_back_ms           */ 350,
	/* search2_turn_deg          */ 60,
	/* rotation_timeout_ms       */ 150,
	/* pid_kp                    */ 0.03f,
	/* pid_ki                    */ 0.04f,
	/* pid_kd                    */ 0.00065f,
	/* oob_turn_mode             */ 0,
};

MiniNeuroV1Params_t NeuroParams = kDefaults;

#define P(field, lo, hi, isint) {#field, &NeuroParams.field, lo, hi, kDefaults.field, isint}
static const MiniNeuroV1ParamInfo_t kTable[] = {
	P(detection_distance_mm,     150,   600,   true),
	P(forward_band_mm,           10,    200,   true),
	P(tof_eval_timeout_ms,       20,    100,   true),
	P(tof_stale_ms,              45,    150,   true),
	P(sig_threshold_gain,        1.8,   4.0,   false),
	P(sig_threshold_cap,         1000,  30000, true),
	P(attack_speed,              0.4,   1.0,   false),
	P(turn_outer_speed,          0.3,   1.0,   false),
	P(turn_inner_speed,          -0.5,  1.0,   false),
	P(search_spin_speed,         0.3,   1.0,   false),
	P(search2_speed,             0.3,   1.0,   false),
	P(oob_back_speed,            0.4,   1.0,   false),
	P(oob_turn_speed,            0.3,   1.0,   false),
	P(oob_stop_ms,               0,     100,   true),
	P(oob_back_ms,               150,   600,   true),
	P(oob_turn_ms,               0,     400,   true),
	P(search_strategy_change_ms, 300,   10000, true),
	P(search2_back_ms,           100,   800,   true),
	P(search2_turn_deg,          15,    180,   false),
	P(rotation_timeout_ms,       50,    600,   true),
	P(pid_kp,                    0.0,   0.15,  false),
	P(pid_ki,                    0.0,   0.15,  false),
	P(pid_kd,                    0.0,   0.005, false),
	P(oob_turn_mode,             0,     1,     true),
};
#undef P

size_t MiniNeuroV1_ParamCount(void)
{
	return sizeof(kTable) / sizeof(kTable[0]);
}

const MiniNeuroV1ParamInfo_t *MiniNeuroV1_ParamInfo(size_t index)
{
	return (index < MiniNeuroV1_ParamCount()) ? &kTable[index] : NULL;
}

bool MiniNeuroV1_SetParam(const char *name, double value)
{
	for (size_t i = 0; i < MiniNeuroV1_ParamCount(); i++)
	{
		const MiniNeuroV1ParamInfo_t *p = &kTable[i];
		if (strcmp(p->name, name) == 0)
		{
			if (value < p->min) value = p->min;
			if (value > p->max) value = p->max;
			*p->value = p->integer ? round(value) : value;
			return true;
		}
	}
	return false;
}

void MiniNeuroV1_ResetParams(void)
{
	NeuroParams = kDefaults;
}
#endif