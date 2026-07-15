/*
 * miniTomi.hpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 */

#ifndef ALGORITHMS_INC_ALGO_THR_HPP_
#define ALGORITHMS_INC_ALGO_THR_HPP_
#include "osapi.h"
#include <cmath>

class Robot : public MortalThread
{
public:
	Robot();
	virtual void PeriodCB();
	virtual void PeriodicCheckCall(void);
	virtual void SetFlashPeriodMS(uint16_t flashPeriod);
	// Heading PID interface: the algorithm sets the target and reads the latest
	// correction; the PID itself runs only in PeriodicCheckCall(), which owns its state.
	void SetTargetHeading(float targetAngleDeg);
	float GetHeadingCorrection(void) { return power_correction; }
	float GetHeadingCorrection(float limit) { return (fabsf(power_correction) > limit) ? (power_correction < 0) ? -limit : limit : power_correction; }
protected:
    void begin(void);
    void loop(void);
    void end(void);
private:
    uint16_t flash_period_ms = 100;
    float set_angle = 0;
    float power_correction = 0;
    bool was_running = false;
};



#endif /* ALGORITHMS_INC_ALGO_THR_HPP_ */
