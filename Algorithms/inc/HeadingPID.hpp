/*
 * HeadingPID.hpp
 *
 * PID controller for holding a yaw heading measured on a wrapping +/-180 deg scale.
 * Understands the two quirks of angular error: the wrap seam (error is always the
 * shortest way around) and the antipode ambiguity (near 180 deg error the turn
 * direction sticks to the previous sign instead of flapping with sensor noise).
 *
 * SetTarget()/SetGains() may be called from a different task/context than Update();
 * the cross-context members are volatile and the target change is handed over with
 * a flag so the integral/derivative history restarts cleanly on the new target.
 */

#ifndef ALGORITHMS_INC_HEADINGPID_HPP_
#define ALGORITHMS_INC_HEADINGPID_HPP_

#include "osapi.h"

class HeadingPID
{
public:
	HeadingPID(float kp, float ki, float kd, float outputLimit, float antipodeHystDeg);
	// New target heading in degrees; resets the integral and suppresses the derivative
	// kick the target jump would otherwise cause on the next Update().
	void SetTarget(float targetAngleDeg);
	float GetTarget(void);
	// Overwrite the gains at runtime (e.g. from a relay auto-tune).
	void SetGains(float kp, float ki, float kd);
	// Steering correction in [-outputLimit, +outputLimit] toward the target, from the
	// current yaw in degrees. Rate-guarded: a second call within the same RTOS tick
	// returns the cached output without recomputing.
	float Update(float currentAngleDeg);
	// Wraps an angle difference into [-180, 180) - the shortest way around the circle.
	static float WrapAngleDeg180(float angleDeg);
private:
	float kp;
	float ki;
	float kd;
	float output_limit;
	float antipode_hyst_deg;
	// Cross-task handshake written by SetTarget(), consumed by Update(): volatile keeps the
	// stores real and ordered (target first, flag second). Everything below is Update()-only.
	volatile float target_angle_deg;
	volatile bool target_changed;
	float integral;
	float last_error;
	TickType_t last_tick;
	float output;
};

#endif /* ALGORITHMS_INC_HEADINGPID_HPP_ */