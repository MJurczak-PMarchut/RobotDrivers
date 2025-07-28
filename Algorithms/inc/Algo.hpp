/*
 * miniTomi.hpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 */

#ifndef ALGORITHMS_INC_ALGO_THR_HPP_
#define ALGORITHMS_INC_ALGO_THR_HPP_
#include "osapi.h"

class Robot : public MortalThread
{
public:
	Robot();
	virtual void PeriodCB();
	virtual void PeriodicCheckCall(void);
	virtual void SetFlashPeriodMS(uint16_t flashPeriod);
protected:
    void begin(void);
    void loop(void);
    void end(void);
private:
    uint16_t flash_period_ms = 100;
    bool was_running = false;
};



#endif /* ALGORITHMS_INC_ALGO_THR_HPP_ */
