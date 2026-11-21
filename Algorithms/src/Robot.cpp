/*
 * Robot.cpp
 *
 * Default implementations of the Robot base class (Algo.hpp). Compiled for every robot:
 * Robot::Calibrate() is the first non-inline virtual method of Robot, so the vtable of Robot
 * is emitted here.
 */
#include "stm32h7xx_hal.h"
#include "Algo.hpp"
#if defined(ROBOT_MT_V4) || defined(ROBOT_MT_V5) || defined(ROBOT_MINI_NEURO_V1) || defined(ROBOT_MINI_NEURO_V2)
#define ROBOT_HAS_SHARED_HARDWARE
#include <algorithm>
#include <cstdio>
#include <iterator>
#include "RobotHardware.hpp"
#endif

// MiniTomiV5's calibration, the default (MiniTomiV4 uses it too). The display update after
// it (DEBUG_SCREEN) stays in the strategy's loop(), because the stats boxes belong to the
// strategy. Robots without the shared hardware (MiniTomiV2/V3) override it, so there the
// default is empty.
void Robot::Calibrate(void)
{
#ifdef ROBOT_HAS_SHARED_HARDWARE
	TickType_t start_time = xTaskGetTickCount();
	while((xTaskGetTickCount() - start_time) < kCalibrationMs){
		xEventGroupWaitBits(ToF_Sensor::GetEventHandle(), TOF_EVENT_MASK, pdTRUE, pdTRUE, 30);
		for (std::size_t sensor = 0; sensor < std::size(Sensors); sensor++){
			VL53L1X_Result_t Result = Sensors[sensor].GetResult();
			SigPerSPADThreshold[sensor] = (SigPerSPADThreshold[sensor] < Result.SigPerSPAD)? Result.SigPerSPAD: SigPerSPADThreshold[sensor];
		}
	}
	for (std::size_t sensor = 0; sensor < std::size(Sensors); sensor++){
		// Double and clamp in 32 bits: a uint16_t threshold above 32767 would wrap when doubled.
		SigPerSPADThreshold[sensor] = std::min<uint32_t>(SigPerSPADThreshold[sensor] * 2u, kSigThresholdCap);
	}
	char calib_message[100];
	sprintf(calib_message, "Calib SigPerSPAD thresholds L=%u R=%u B=%u", SigPerSPADThreshold[0], SigPerSPADThreshold[2], SigPerSPADThreshold[1]);
	logger.Log(calib_message, LOGLEVEL_INFO);
	logger.Log("--- CALIBRATION Completed ---", LOGLEVEL_INFO);
#endif
}

// MiniTomiV5's validated ToF readings, the default (MiniTomiV4 uses it too). Reads all
// sensors and replaces readings whose signal is weaker than the calibrated threshold with an
// out-of-detection-range value, so callers only act on distances the calibration deems
// trustworthy. Stale sensors are treated the same. Robots without the shared hardware
// (MiniTomiV2/V3) never call it, so there it returns zeros.
SensorDistances_t Robot::GetValidatedDistances(void)
{
#ifdef ROBOT_HAS_SHARED_HARDWARE
	SensorDistances_t distances;
	ToFReading_t* readings[] = {&distances.left, &distances.rear, &distances.right};
	static_assert(std::size(readings) == std::size(Sensors));
	uint32_t now = HAL_GetTick();
	// Same order as Sensors[]: 0 = left, 1 = rear, 2 = right
	for(std::size_t sensor = 0; sensor < std::size(Sensors); sensor++){
		ToFReading_t& r = *readings[sensor];
		// Signed age: an ISR may store a tick newer than `now` between the two reads (B16).
		r.stale = (int32_t)(now - Sensors[sensor].GetLastSampleTick()) > kTofStaleMs;
		r.raw = Sensors[sensor].GetDistance();
		r.sig = Sensors[sensor].GetResult().SigPerSPAD;
		r.filtered = (!r.stale && r.sig > SigPerSPADThreshold[sensor])? r.raw: kDetectionDistanceMm + 100;
	}
	return distances;
#else
	return {};
#endif
}

// Default out-of-bounds check on the board's line sensors (LineSensors, RobotHardware.hpp).
// Robots without the shared hardware have no LineSensors: MiniTomiV3 overrides it, MiniTomiV2
// reads its pins itself, so there it is never on the line.
bool Robot::isOutOfBounds(void)
{
#ifdef ROBOT_HAS_SHARED_HARDWARE
	const uint32_t on_line = LineSensors.Read();
	if(on_line != 0)
	{
		line_det = on_line;
	}
	return on_line != 0;
#else
	return false;
#endif
}

// True if a sensor in `mask` is mounted on `side` of the centre line (+1 left, -1 right).
static bool AnyLineSensorOnSide(uint32_t mask, int side)
{
#ifdef ROBOT_HAS_SHARED_HARDWARE
	for(std::size_t i = 0; i < LineSensors.Count(); i++)
	{
		if((mask & (1u << i)) != 0 && LineSensors.Mount(i).y_mm * side > 0)
		{
			return true;
		}
	}
#else
	(void)mask;
	(void)side;
#endif
	return false;
}

bool Robot::LineDetOnLeft(void) const
{
	return AnyLineSensorOnSide(line_det, +1);
}

bool Robot::LineDetOnRight(void) const
{
	return AnyLineSensorOnSide(line_det, -1);
}
