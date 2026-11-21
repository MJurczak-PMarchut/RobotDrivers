/*
 * miniTomi.hpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 *
 * Robot is the base class of every strategy; Core/Src/DebugEntryCPP.cpp creates the one
 * selected by the build's ROBOT_* define and drives it through this interface (run(),
 * PeriodCB(), PeriodicCheckCall()). Each strategy is a class derived from Robot, declared and
 * implemented with its own code and created by its factory below.
 */

#ifndef ALGORITHMS_INC_ALGO_THR_HPP_
#define ALGORITHMS_INC_ALGO_THR_HPP_
#include "osapi.h"
#include "HeadingPID.hpp"
#include <cmath>
#include <cstdint>

// One ToF sensor reading, as returned by Robot::GetValidatedDistances().
typedef struct{
	uint16_t raw;		// odczyt prosto z czujnika
	uint16_t filtered;	// raw albo DETECTION_DISTANCE + 100 (stale / za słaby sygnał)
	uint16_t sig;		// SigPerSPAD porównywany z SigPerSPADThreshold
	bool stale;			// brak próbki przez TOF_STALE_MS
}ToFReading_t;

// All ToF sensors, in the order of Sensors[] (RobotHardware.hpp): 0 = left, 1 = rear, 2 = right.
typedef struct{
	ToFReading_t left;
	ToFReading_t rear;
	ToFReading_t right;
}SensorDistances_t;

class Robot : public MortalThread
{
public:
	Robot() : MortalThread(tskIDLE_PRIORITY, 4096, "Main Algo task") {}
	virtual void PeriodCB(void) = 0;
	virtual void PeriodicCheckCall(void) = 0;
	void SetFlashPeriodMS(uint16_t flashPeriod) { flash_period_ms = flashPeriod; }
	// Heading control interface: the algorithm sets the target and reads the latest
	// correction; the controller itself runs only in PeriodicCheckCall()
	// (UpdateHeadingCorrection()), which owns its state. Virtual: the strategy helpers call it
	// through Robot*; a strategy with its own controller overrides it together with
	// UpdateHeadingCorrection().
	virtual void SetTargetHeading(float targetAngleDeg)
	{
		set_angle = targetAngleDeg;
		heading_pid.SetTarget(targetAngleDeg);
	}
	float GetHeadingCorrection(void) { return power_correction; }
	float GetHeadingCorrection(float limit) { return (fabsf(power_correction) > limit) ? (power_correction < 0) ? -limit : limit : power_correction; }
	// Out of bounds: true if any line sensor sees the line now; the sensors that do are latched
	// in line_det. The default (Robot.cpp) reads LineSensors (RobotHardware.hpp). Public: the
	// strategy helpers call it through Robot*. A strategy that also reacts to the edge
	// (MiniNeuroV2) overrides it and calls the default.
	virtual bool isOutOfBounds(void);
	// begin() / loop() / end(): from MortalThread, implemented by each strategy.
protected:
    uint16_t flash_period_ms = 100;
    float set_angle = 0;
    float power_correction = 0;
    bool was_running = false;

	// Line sensors that saw the line at the last isOutOfBounds() == true: bit i = LineSensors[i].
	// Latched because the sensors may already read clear by the time the OOB handler logs them.
	uint32_t line_det = 0;
	// Side of the latched sensors (LineSensors mount: y > 0 left, y < 0 right).
	bool LineDetOnLeft(void) const;
	bool LineDetOnRight(void) const;

	// Boot-time calibration, called from the strategy's loop(). The default (Robot.cpp) is
	// MiniTomiV5's ToF calibration, also used by MiniTomiV4: for kCalibrationMs keep the largest
	// SigPerSPAD of every sensor, then SigPerSPADThreshold[i] = min(2 x max, kSigThresholdCap).
	// Strategies with other rules (MiniTomiV2/V3, MiniNeuroV1/V2) override it.
	virtual void Calibrate(void);
	static constexpr uint32_t kCalibrationMs = 150;
	static constexpr uint32_t kSigThresholdCap = 8000;

	// Latest reading of every ToF sensor, each range gated by its age and by
	// SigPerSPADThreshold[]. The default (Robot.cpp) is MiniTomiV5's, also used by MiniTomiV4:
	// a range older than kTofStaleMs or with SigPerSPAD not above the threshold becomes
	// kDetectionDistanceMm + 100 (out of detection range). Strategies with other rules
	// (MiniNeuroV1/V2: limits from their params) override it.
	virtual SensorDistances_t GetValidatedDistances(void);
	static constexpr uint16_t kDetectionDistanceMm = 350;
	// A sensor without a sample for this long (~2 periods) counts as "no detection".
	static constexpr int32_t kTofStaleMs = 60;

	// Heading controller step: PeriodicCheckCall() calls it with the current yaw (deg); it sets
	// power_correction (GetHeadingCorrection()). The default runs heading_pid.
	virtual void UpdateHeadingCorrection(float yawDeg) { power_correction = heading_pid.Update(yawDeg); }

	// Default heading-hold PID, used by SetTargetHeading() and UpdateHeadingCorrection(). A
	// strategy may change its gains at any time with heading_pid.SetGains() (in its constructor,
	// in begin(), or after an auto-tune such as mt_v4::PID_CalibrateHeading()), or replace the
	// controller altogether by overriding SetTargetHeading() and UpdateHeadingCorrection().
	static constexpr float kHeadingKp = 0.03f;
	static constexpr float kHeadingKi = 0.04f;
	static constexpr float kHeadingKd = 0.00065f;
	static constexpr float kHeadingOutputLimit = 0.75f;
	// Near a 180-deg error the shortest turn direction is ambiguous and yaw noise flips the
	// wrapped error between +-180 every sample. Within this band of the antipode the PID
	// sticks to its previous turn direction; outside it the plain shortest-way error rules,
	// so the worst case is turning (180 + this) deg instead of (180 - this).
	static constexpr float kHeadingAntipodeHystDeg = 10.0f;
	HeadingPID heading_pid{kHeadingKp, kHeadingKi, kHeadingKd, kHeadingOutputLimit, kHeadingAntipodeHystDeg};
};

/*
 * Strategy factories, one per ROBOT_* define (DebugEntryCPP.cpp calls the one of the build).
 * Each creates its strategy with new; the caller owns the object. The strategy classes are
 * declared next to their code (e.g. MiniTomiV4 in MiniTomiV4Helpers.hpp, MiniTomiV2 in
 * miniTomiV2.cpp), so this header stays free of strategy internals and of their macros/types.
 */
Robot* CreateMiniTomiV2(void);
Robot* CreateMiniTomiV3(void);
Robot* CreateMiniTomiV4(void);
Robot* CreateMiniTomiV5(void);
Robot* CreateMiniNeuroV1(void);
Robot* CreateMiniNeuroV2(void);

#endif /* ALGORITHMS_INC_ALGO_THR_HPP_ */
