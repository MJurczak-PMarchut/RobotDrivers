/*
 * MiniNeuroV2Params.cpp
 *
 * Defaults and ranges of the miniNeuroV2 parameters (see MiniNeuroV2Params.hpp). The ranges
 * bound what the optimiser in SumoSimulation may try. The V1 part keeps miniNeuroV1's ranges
 * (see MiniNeuroV1Params.cpp for why some lower limits are there).
 */
#ifdef ROBOT_MINI_NEURO_V2
#include "MiniNeuroV2Params.hpp"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Defaults = miniTomiV4 constants (to be replaced by the tuned miniNeuroV1 values), the V1
// FSM in charge (policy_mode 0), full history, rear sensor on.
static const MiniNeuroV2Params_t kDefaults = {
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
	/* policy_mode               */ 0,
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
