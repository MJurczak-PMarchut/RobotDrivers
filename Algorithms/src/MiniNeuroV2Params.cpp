/*
 * MiniNeuroV2Params.cpp
 *
 * Defaults and ranges of the miniNeuroV2 parameters (see MiniNeuroV2Params.hpp). The ranges
 * bound what the optimiser in SumoSimulation may try. The V1 part keeps miniNeuroV1's ranges
 * (see MiniNeuroV1Params.cpp for why some lower limits are there).
 */
#if defined(ROBOT_MINI_NEURO_V2) || defined(ROBOT_ALGO_SELECT)   // ROBOT_ALGO_SELECT: all strategies in one image
#include "MiniNeuroV2Params.hpp"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Defaults = SumoSimulation run 10 (2026-11-20): the FSM parameters of the tuned miniNeuroV1
// teacher (runs/neuro1_v10, search2_back_ms 350 and detection_distance_mm 450 set by hand) and
// the network in charge (policy_mode 1) with the weights in MiniNeuroV2Weights.hpp
// (runs/neuro2_evo_v10). policy_mode 0 makes the V1 FSM fight with the same parameters.
static const MiniNeuroV2Params_t kDefaults = {
	// from runs/neuro2_evo_v10/best_params.txt (tools/neuro_params_to_cpp.py)
	/* detection_distance_mm     */ 450,
	/* forward_band_mm           */ 76,
	/* tof_eval_timeout_ms       */ 30,
	/* tof_stale_ms              */ 60,
	/* sig_threshold_gain        */ 1.81216743,
	/* sig_threshold_cap         */ 19853,
	/* attack_speed              */ 0.843932595,
	/* turn_outer_speed          */ 0.997027994,
	/* turn_inner_speed          */ -0.280367351,
	/* search_spin_speed         */ 0.526142318,
	/* search2_speed             */ 0.786338367,
	/* oob_back_speed            */ 0.404957866,
	/* oob_turn_speed            */ 0.410835363,
	/* oob_stop_ms               */ 21,
	/* oob_back_ms               */ 178,
	/* oob_turn_ms               */ 76,
	/* search_strategy_change_ms */ 9522,
	/* search2_back_ms           */ 350,
	/* search2_turn_deg          */ 126.815868,
	/* rotation_timeout_ms       */ 279,
	/* pid_kp                    */ 0.03,
	/* pid_ki                    */ 0.04,
	/* pid_kd                    */ 0.00065,
	/* oob_turn_mode             */ 1,
	/* policy_mode               */ 1,
	/* net_history               */ 3,
	/* net_use_rear              */ 1,
};

MiniNeuroV2Params_t Neuro2Params = kDefaults;

// Weights start from the built-in defaults; constant-initialised arrays are ready before
// any dynamic initialiser runs, and on the simulator before simfw_set_param.
static struct Neuro2WeightsInit {
	Neuro2WeightsInit() { memcpy(Neuro2Weights, kNeuro2DefaultWeights, sizeof(Neuro2Weights)); }
} s_weights_init;

// Weights may go beyond this in principle; the range keeps the optimiser's box finite.
#define NEURO2_WEIGHT_LIMIT 8.0

#define P(field, lo, hi, isint) {#field, &Neuro2Params.field, NULL, lo, hi, kDefaults.field, isint}
static const MiniNeuroV2ParamInfo_t kTable[] = {
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
	P(policy_mode,               0,     1,     true),
	P(net_history,               1,     3,     true),
	P(net_use_rear,              0,     1,     true),
};
#undef P
#define TABLE_SIZE (sizeof(kTable) / sizeof(kTable[0]))

size_t MiniNeuroV2_ParamCount(void)
{
	return TABLE_SIZE + NEURO2_WEIGHTS;
}

static void WeightName(size_t w, char name[8])
{
	snprintf(name, 8, "nn_w%03u", (unsigned)w);
}

const MiniNeuroV2ParamInfo_t *MiniNeuroV2_ParamInfo(size_t index)
{
	if (index < TABLE_SIZE)
	{
		return &kTable[index];
	}
	if (index >= TABLE_SIZE + NEURO2_WEIGHTS)
	{
		return NULL;
	}
	static char name[8];
	static MiniNeuroV2ParamInfo_t weight;
	const size_t w = index - TABLE_SIZE;
	WeightName(w, name);
	weight = {name, NULL, &Neuro2Weights[w], -NEURO2_WEIGHT_LIMIT, NEURO2_WEIGHT_LIMIT,
			(double)kNeuro2DefaultWeights[w], false};
	return &weight;
}

bool MiniNeuroV2_SetParam(const char *name, double value)
{
	// nn_wNNN: index straight from the name.
	if (strncmp(name, "nn_w", 4) == 0)
	{
		char *end = NULL;
		const unsigned long w = strtoul(name + 4, &end, 10);
		char canonical[8];
		if (w < NEURO2_WEIGHTS && end && *end == '\0')
		{
			WeightName(w, canonical);
			if (strcmp(canonical, name) == 0)
			{
				if (value < -NEURO2_WEIGHT_LIMIT) value = -NEURO2_WEIGHT_LIMIT;
				if (value > NEURO2_WEIGHT_LIMIT) value = NEURO2_WEIGHT_LIMIT;
				Neuro2Weights[w] = (float)value;
				return true;
			}
		}
		return false;
	}
	for (size_t i = 0; i < TABLE_SIZE; i++)
	{
		const MiniNeuroV2ParamInfo_t *p = &kTable[i];
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

void MiniNeuroV2_ResetParams(void)
{
	Neuro2Params = kDefaults;
	memcpy(Neuro2Weights, kNeuro2DefaultWeights, sizeof(Neuro2Weights));
}
#endif
