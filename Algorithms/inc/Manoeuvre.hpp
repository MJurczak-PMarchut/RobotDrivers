/*
 * Manoeuvre.hpp
 *
 * Non-blocking timed manoeuvre shared by the strategies (MiniTomiV4/V5, MiniNeuroV1/V2): a
 * state steps it on every loop() wake-up, and the loop shortens its wait to the phase
 * deadline (WaitTicks()). Reset on every state entry (End()).
 */

#ifndef ALGORITHMS_INC_MANOEUVRE_HPP_
#define ALGORITHMS_INC_MANOEUVRE_HPP_

#include <cstdint>
#include "FreeRTOS.h"
#include "task.h"

// Phase 0 = no manoeuvre; strategies number their phases from MANOEUVRE_IDLE + 1.
inline constexpr uint8_t MANOEUVRE_IDLE = 0;

struct Manoeuvre
{
	bool active = false;
	uint8_t phase = MANOEUVRE_IDLE;
	TickType_t deadline = 0;
	float start_heading = 0;

	void EnterPhase(uint8_t newPhase, uint32_t duration_ms)
	{
		active = true;
		phase = newPhase;
		deadline = xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);
	}

	void End(void)
	{
		active = false;
		phase = MANOEUVRE_IDLE;
	}

	bool PhaseDone(void) const
	{
		// Signed difference: correct across tick-counter wrap.
		return active && ((int32_t)(xTaskGetTickCount() - deadline) >= 0);
	}

	TickType_t WaitTicks(TickType_t max_ticks) const
	{
		if(!active)
		{
			return max_ticks;
		}
		int32_t left = (int32_t)(deadline - xTaskGetTickCount());
		if(left <= 0)
		{
			return 0;
		}
		return ((TickType_t)left < max_ticks)? (TickType_t)left : max_ticks;
	}
};

#endif /* ALGORITHMS_INC_MANOEUVRE_HPP_ */
