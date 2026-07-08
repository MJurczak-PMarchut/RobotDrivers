/*
 * lsm6dso_quat.cpp
 */
#include "lsm6dso_quat.hpp"

#define G_TO_MPS2 9.80665
#define DEG_TO_RAD_FACTOR 0.017453292519943295

// Proportional gain pulling roll/pitch back toward the accelerometer's gravity vector; needs
// tuning on real hardware (too high fights genuine rotation, too low barely corrects drift).
#define MAHONY_TWO_KP 0.5

// Only trust the accelerometer for tilt correction close to a resting 1g; further out (a
// collision, hard acceleration) the reading no longer isolates gravity's direction.
#define TILT_CORRECTION_ACCEL_TOLERANCE_MG 200.0

LSM6DSOQuat::LSM6DSOQuat()
{
	this->q[0] = 1.0;
	this->q[1] = 0.0;
	this->q[2] = 0.0;
	this->q[3] = 0.0;
	for(uint8_t axis=0; axis < 3; axis++){
		this->velocity_mps[axis] = 0;
		this->position_m[axis] = 0;
	}
}

void LSM6DSOQuat::NormalizeQuaternion(void)
{
	double_t norm = sqrt(this->q[0]*this->q[0] + this->q[1]*this->q[1]
			+ this->q[2]*this->q[2] + this->q[3]*this->q[3]);
	if(norm > 0.0){
		this->q[0] /= norm;
		this->q[1] /= norm;
		this->q[2] /= norm;
		this->q[3] /= norm;
	}
}

void LSM6DSOQuat::Update(double_t gx_mdps, double_t gy_mdps, double_t gz_mdps,
		double_t ax_mg, double_t ay_mg, double_t az_mg, double_t dt_s)
{
#if LSM6DSO_QUAT_ESTIMATION_ENABLED
	double_t gx = (gx_mdps / 1000.0) * DEG_TO_RAD_FACTOR;
	double_t gy = (gy_mdps / 1000.0) * DEG_TO_RAD_FACTOR;
	double_t gz = (gz_mdps / 1000.0) * DEG_TO_RAD_FACTOR;

	double_t q0 = this->q[0], q1 = this->q[1], q2 = this->q[2], q3 = this->q[3];

	double_t accel_mag_mg = sqrt(ax_mg*ax_mg + ay_mg*ay_mg + az_mg*az_mg);
	if(accel_mag_mg > 0.0 && fabs(accel_mag_mg - 1000.0) < TILT_CORRECTION_ACCEL_TOLERANCE_MG){
		// Normalised measured direction of gravity, in body frame.
		double_t ax = ax_mg / accel_mag_mg;
		double_t ay = ay_mg / accel_mag_mg;
		double_t az = az_mg / accel_mag_mg;

		// Estimated direction of gravity in body frame, from the current attitude estimate.
		double_t halfvx = q1*q3 - q0*q2;
		double_t halfvy = q0*q1 + q2*q3;
		double_t halfvz = q0*q0 - 0.5 + q3*q3;

		// Error is the rotation needed to align the estimate with the measurement.
		double_t halfex = (ay*halfvz - az*halfvy);
		double_t halfey = (az*halfvx - ax*halfvz);
		double_t halfez = (ax*halfvy - ay*halfvx);

		gx += MAHONY_TWO_KP * halfex;
		gy += MAHONY_TWO_KP * halfey;
		gz += MAHONY_TWO_KP * halfez;
	}

	// Integrate rate of change of quaternion.
	gx *= 0.5 * dt_s;
	gy *= 0.5 * dt_s;
	gz *= 0.5 * dt_s;
	double_t qa = q0, qb = q1, qc = q2;
	q0 += (-qb*gx - qc*gy - q3*gz);
	q1 += ( qa*gx + qc*gz - q3*gy);
	q2 += ( qa*gy - qb*gz + q3*gx);
	q3 += ( qa*gz + qb*gy - qc*gx);

	this->q[0] = q0;
	this->q[1] = q1;
	this->q[2] = q2;
	this->q[3] = q3;
	NormalizeQuaternion();
	q0 = this->q[0]; q1 = this->q[1]; q2 = this->q[2]; q3 = this->q[3];

	// Rotate the raw (gravity-included) body-frame accel into the world frame using the
	// now-updated attitude, then remove world-frame gravity (+1g along world Z) before
	// double-integrating the remaining linear acceleration into velocity and position.
	double_t ax_g = ax_mg / 1000.0;
	double_t ay_g = ay_mg / 1000.0;
	double_t az_g = az_mg / 1000.0;

	double_t world_ax = (1 - 2*(q2*q2 + q3*q3))*ax_g + 2*(q1*q2 - q0*q3)*ay_g + 2*(q1*q3 + q0*q2)*az_g;
	double_t world_ay = 2*(q1*q2 + q0*q3)*ax_g + (1 - 2*(q1*q1 + q3*q3))*ay_g + 2*(q2*q3 - q0*q1)*az_g;
	double_t world_az = 2*(q1*q3 - q0*q2)*ax_g + 2*(q2*q3 + q0*q1)*ay_g + (1 - 2*(q1*q1 + q2*q2))*az_g;

	double_t linear_ax_mps2 = world_ax * G_TO_MPS2;
	double_t linear_ay_mps2 = world_ay * G_TO_MPS2;
	double_t linear_az_mps2 = (world_az - 1.0) * G_TO_MPS2;

	this->velocity_mps[0] += linear_ax_mps2 * dt_s;
	this->velocity_mps[1] += linear_ay_mps2 * dt_s;
	this->velocity_mps[2] += linear_az_mps2 * dt_s;
	this->position_m[0] += this->velocity_mps[0] * dt_s;
	this->position_m[1] += this->velocity_mps[1] * dt_s;
	this->position_m[2] += this->velocity_mps[2] * dt_s;
#else
	(void)gx_mdps; (void)gy_mdps; (void)gz_mdps;
	(void)ax_mg; (void)ay_mg; (void)az_mg; (void)dt_s;
#endif
}

double_t LSM6DSOQuat::GetOrientationForAxis(uint8_t axis)
{
	double_t q0 = this->q[0], q1 = this->q[1], q2 = this->q[2], q3 = this->q[3];
	double_t angle_rad;
	switch(axis){
	case 0: // roll (X)
		angle_rad = atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
		break;
	case 1: // pitch (Y)
	{
		double_t sinp = 2.0*(q0*q2 - q3*q1);
		// Clamp against numeric overshoot; at |sinp|=1 the robot is pointing straight up/down
		// (gimbal lock of the ZYX extraction - the quaternion itself is unaffected).
		if(sinp > 1.0)
			sinp = 1.0;
		else if(sinp < -1.0)
			sinp = -1.0;
		angle_rad = asin(sinp);
	}
		break;
	case 2: // yaw (Z)
		angle_rad = atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));
		break;
	default:
		return NAN;
	}
	return angle_rad / DEG_TO_RAD_FACTOR;
}

PositionTypeDef LSM6DSOQuat::GetPosition(void)
{
	PositionTypeDef pos;
	pos.x = this->position_m[0];
	pos.y = this->position_m[1];
	pos.z = this->position_m[2];
	return pos;
}

void LSM6DSOQuat::CalibratePosition(void)
{
	for(uint8_t axis=0; axis < 3; axis++){
		this->position_m[axis] = 0;
	}
}
