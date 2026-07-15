/*
 * HeadingPID.cpp
 */
#include "HeadingPID.hpp"
#include <math.h>

#define MIN_DEGREE_CHANGE  5.0f

HeadingPID::HeadingPID(float kp, float ki, float kd, float outputLimit, float antipodeHystDeg)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->output_limit = outputLimit;
	this->antipode_hyst_deg = antipodeHystDeg;
	this->target_angle_deg = 0.0f;
	this->integral = 0.0f;
	this->target_changed = false;
	this->last_error = 0.0f;
	this->last_tick = 0;
	this->output = 0.0f;
}

float HeadingPID::WrapAngleDeg180(float angleDeg)
{
	angleDeg = fmodf(angleDeg + 180.0f, 360.0f);
	if (angleDeg < 0.0f)
	{
		angleDeg += 360.0f;
	}
	return angleDeg - 180.0f;
}

void HeadingPID::SetTarget(float targetAngleDeg)
{
	// Only publish the target and the flag here; the integral/derivative reset happens in
	// Update() when it consumes the flag, so a SetTarget() preempting a running Update()
	// can't have its state reset overwritten by Update()'s final stores.
	float previousTarget = this->target_angle_deg;
	this->target_angle_deg = targetAngleDeg;
	if (fabs(WrapAngleDeg180(previousTarget - targetAngleDeg)) > MIN_DEGREE_CHANGE)
	{
		this->target_changed = true;
	}
}

float HeadingPID::GetTarget(void)
{
	return this->target_angle_deg;
}

void HeadingPID::SetGains(float kp, float ki, float kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

float HeadingPID::Update(float currentAngleDeg)
{
	TickType_t now = xTaskGetTickCount();
	if (this->last_tick == now)
	{
		return this->output;
	}
	float dt = (this->last_tick == 0) ? 0.0f : (float)(now - this->last_tick) / 1000.0f;
	this->last_tick = now;

	float error = WrapAngleDeg180(this->target_angle_deg - currentAngleDeg);

	if (this->target_changed)
	{
		this->last_error = error; // avoid a derivative kick from the previous target's error
		this->integral = 0.0f;    // a fresh target starts with a clean integral
	}
	// Antipode hysteresis: inside the band the turn direction follows the previous error's
	// sign instead of the noise-flipped wrapped sign; outside the band the plain shortest-way
	// error always wins, so a disturbance that genuinely shortens the other way takes over.
	else if (error > (180.0f - this->antipode_hyst_deg) && this->last_error < 0.0f)
	{
		error -= 360.0f;
	}
	else if (error < -(180.0f - this->antipode_hyst_deg) && this->last_error > 0.0f)
	{
		error += 360.0f;
	}
	this->target_changed = false;

	float derivative = (dt > 0.0f) ? (error - this->last_error) / dt : 0.0f;
	this->last_error = error;
	float integral_tmp = error * dt;
	float integral_new = this->integral + integral_tmp;
	float integralLimit = (this->ki > 0.0f) ? 2.0f * (this->output_limit / this->ki) : 0.0f;
	if (integral_new > integralLimit)
	{
		integral_new = integralLimit;
	}
	else if (integral_new < -integralLimit)
	{
		integral_new = -integralLimit;
	}

	float output_new = this->kp * error + this->ki * integral_new + this->kd * derivative;

	if (output_new > this->output_limit)
	{
		output_new = this->output_limit;
		integral_new -= integral_tmp; // anti-windup
	}
	else if (output_new < -this->output_limit)
	{
		output_new = -this->output_limit;
		integral_new -= integral_tmp; // anti-windup
	}

	this->integral = integral_new;
	this->output = output_new;
	return output_new;
}