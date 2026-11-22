/*
 * MiniNeuroV2Net.cpp
 *
 * Forward pass of the miniNeuroV2 network (layout in MiniNeuroV2Net.hpp). About 300
 * multiply-adds and 18 tanhf per decision: a few microseconds on the Cortex-M7.
 */
#if defined(ROBOT_MINI_NEURO_V2) || defined(ROBOT_ALGO_SELECT)   // ROBOT_ALGO_SELECT: all strategies in one image
#include "MiniNeuroV2Net.hpp"
#include <math.h>

#include "MiniNeuroV2Weights.hpp"

float Neuro2Weights[NEURO2_WEIGHTS];

void Neuro2Forward(const float in[NEURO2_INPUTS], float out[NEURO2_OUTPUTS])
{
	float hidden[NEURO2_HIDDEN];
	for (int h = 0; h < NEURO2_HIDDEN; h++)
	{
		const float *w = &Neuro2Weights[NEURO2_HIDDEN_OFFSET + h * (NEURO2_INPUTS + 1)];
		float sum = w[NEURO2_INPUTS];
		for (int i = 0; i < NEURO2_INPUTS; i++)
		{
			sum += w[i] * in[i];
		}
		hidden[h] = tanhf(sum);
	}
	for (int o = 0; o < NEURO2_OUTPUTS; o++)
	{
		const float *w = &Neuro2Weights[NEURO2_OUTPUT_OFFSET + o * (NEURO2_HIDDEN + 1)];
		float sum = w[NEURO2_HIDDEN];
		for (int h = 0; h < NEURO2_HIDDEN; h++)
		{
			sum += w[h] * hidden[h];
		}
		out[o] = tanhf(sum);
	}
}
#endif
