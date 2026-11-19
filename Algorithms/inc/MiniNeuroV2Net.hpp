/*
 * MiniNeuroV2Net.hpp
 *
 * The neural network of miniNeuroV2: a small multilayer perceptron that drives the motors
 * while the robot fights and searches (FSM state NEURAL). The FSM keeps calibration, the
 * start and the line escape; the network cannot switch them off.
 *
 *   inputs (NEURO2_INPUTS) -> tanh hidden layer (NEURO2_HIDDEN) -> tanh outputs (left, right power)
 *
 * The inputs are built by Neuro2Features() (MiniNeuroV2Helpers.cpp), all roughly in [-1, 1]:
 * ToF ranges after the SigPerSPAD gate (0 = touching, 1 = 1000 mm or more / no valid target),
 * with the last samples of each sensor, a short memory of the last detection, the IMU and
 * the previous command.
 *
 * Weights (NEURO2_WEIGHTS floats), row-major:
 *   hidden unit h: w[h * (IN + 1) + i] for input i, bias w[h * (IN + 1) + IN]
 *   output o:      w[HO + o * (HID + 1) + h] for hidden unit h, bias w[HO + o * (HID + 1) + HID]
 *   with HO = NEURO2_HIDDEN * (NEURO2_INPUTS + 1).
 * They are run-time parameters nn_w000.. (MiniNeuroV2Params.cpp); the built-in defaults come
 * from MiniNeuroV2Weights.hpp, generated from a training run (SumoSimulation/tools).
 */

#ifndef ALGORITHMS_INC_MININEUROV2NET_HPP_
#define ALGORITHMS_INC_MININEUROV2NET_HPP_

#define NEURO2_INPUTS  16
#define NEURO2_HIDDEN  16
#define NEURO2_OUTPUTS 2
#define NEURO2_HIDDEN_OFFSET 0
#define NEURO2_OUTPUT_OFFSET (NEURO2_HIDDEN * (NEURO2_INPUTS + 1))
#define NEURO2_WEIGHTS (NEURO2_OUTPUT_OFFSET + NEURO2_OUTPUTS * (NEURO2_HIDDEN + 1))

// Input layout. Keep NEURO2_INPUT_NAMES in the same order.
enum {
	NN_IN_FL0 = 0,          // front-left range now
	NN_IN_FL1,              //   one sample ago (0 when net_history < 2)
	NN_IN_FL2,              //   two samples ago (0 when net_history < 3)
	NN_IN_FR0,              // front-right, same
	NN_IN_FR1,
	NN_IN_FR2,
	NN_IN_RR0,              // rear range now (0 when net_use_rear = 0)
	NN_IN_RR1,              //   one sample ago
	NN_IN_SINCE_DET,        // time since the last front detection: 0..1 = 0..2 s (1 = never)
	NN_IN_DET_SIDE,         // side of that detection: +1 left, -1 right, 0 none yet
	NN_IN_TURNED,           // heading change since that detection, deg / 180 (CCW positive)
	NN_IN_GYRO,             // yaw rate, dps / 720, clipped
	NN_IN_ACCEL_X,          // forward acceleration, g / 2, clipped
	NN_IN_PREV_LEFT,        // previous motor command
	NN_IN_PREV_RIGHT,
	NN_IN_SINCE_START,      // 0..1 = 0..3 s after the start signal
	NN_IN_COUNT
};
static_assert(NN_IN_COUNT == NEURO2_INPUTS, "input layout and NEURO2_INPUTS differ");

#define NEURO2_INPUT_NAMES "fl0,fl1,fl2,fr0,fr1,fr2,rr0,rr1,since_det,det_side,turned,gyro,accel_x,prev_left,prev_right,since_start"

// Current weights (run-time parameters).
extern float Neuro2Weights[NEURO2_WEIGHTS];
// Built-in defaults (MiniNeuroV2Weights.hpp).
extern const float kNeuro2DefaultWeights[NEURO2_WEIGHTS];

// out = tanh(W2 * tanh(W1 * in + b1) + b2), so each output is in (-1, 1).
void Neuro2Forward(const float in[NEURO2_INPUTS], float out[NEURO2_OUTPUTS]);

#endif /* ALGORITHMS_INC_MININEUROV2NET_HPP_ */
