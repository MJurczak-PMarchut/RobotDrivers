/*
 * StrategyTapSelector.hpp
 *
 * Choosing the strategy of the multi-algorithm firmware by tapping the robot while it waits
 * for the start (design: ~/Projekty/MultiAlgorithmV1WorkState.md, "Wybór strategii
 * puknięciami"). Pure logic, no HAL: the caller feeds it the taps counted since the last call
 * and acts on the returned event (screen, storing the choice, restart); see
 * Robot::StrategySelectionStep() in Robot.cpp.
 *
 * Gesture:
 *  - Taps less than kSeriesGapMs apart form a series; a series ends after kSeriesGapMs
 *    without a tap.
 *  - Selection mode is off by default; a series of 3 turns it on (stray taps while putting the
 *    robot on the ring or turning it by hand do nothing).
 *  - In the mode: 1 tap = next strategy, 2 = previous (kStrategyOrder, wrapping), 3 = leave
 *    without a change; other series are ignored.
 *  - kCommitIdleMs without a tap in the mode: a strategy other than the running one is
 *    committed (stored, restart), the running one just leaves the mode.
 */

#ifndef ALGORITHMS_INC_STRATEGYTAPSELECTOR_HPP_
#define ALGORITHMS_INC_STRATEGYTAPSELECTOR_HPP_

#ifdef ROBOT_IS_MULTI_ALGORITHM

#include <cstddef>
#include <cstdint>
#include <iterator>
#include "StrategySelection.hpp"

class StrategyTapSelector
{
public:
	static constexpr uint32_t kSeriesGapMs = 400;
	static constexpr uint32_t kCommitIdleMs = 4000;
	static constexpr uint8_t kEnterTaps = 3;
	static constexpr uint8_t kNextTaps = 1;
	static constexpr uint8_t kPreviousTaps = 2;
	static constexpr uint8_t kLeaveTaps = 3;

	enum class Event : uint8_t
	{
		None,
		ModeEntered,   // Pending() == the running strategy
		Changed,       // Pending() is the newly chosen strategy
		ModeLeft,      // nothing changes
		Commit,        // store Pending() and restart
	};

	// Selection mode off, no series in progress. `running`: the strategy that runs now.
	void Reset(Strategy running)
	{
		running_ = running;
		pending_ = running;
		active_ = false;
		series_taps_ = 0;
	}

	// now_ms: a free-running millisecond clock (wraps); new_taps: taps since the last call.
	Event Update(uint32_t now_ms, uint8_t new_taps)
	{
		if(new_taps != 0u)
		{
			const unsigned sum = series_taps_ + new_taps;
			series_taps_ = static_cast<uint8_t>((sum > 255u)? 255u : sum);
			last_tap_ms_ = now_ms;
			return Event::None;
		}
		const uint32_t idle_ms = now_ms - last_tap_ms_;
		if(series_taps_ != 0u)
		{
			if(idle_ms < kSeriesGapMs)
			{
				return Event::None;
			}
			const uint8_t taps = series_taps_;
			series_taps_ = 0;
			return SeriesEnded(taps);
		}
		if(active_ && (idle_ms >= kCommitIdleMs))
		{
			active_ = false;
			return (pending_ != running_)? Event::Commit : Event::ModeLeft;
		}
		return Event::None;
	}

	bool Active(void) const { return active_; }
	Strategy Pending(void) const { return pending_; }
	Strategy Running(void) const { return running_; }

	// kStrategyOrder neighbours, wrapping; a strategy missing from the list goes to its start.
	static Strategy Next(Strategy strategy) { return Step(strategy, 1); }
	static Strategy Previous(Strategy strategy) { return Step(strategy, std::size(kStrategyOrder) - 1u); }

private:
	Event SeriesEnded(uint8_t taps)
	{
		if(!active_)
		{
			if(taps != kEnterTaps)
			{
				return Event::None;
			}
			active_ = true;
			pending_ = running_;
			return Event::ModeEntered;
		}
		switch(taps)
		{
		case kNextTaps:
			pending_ = Next(pending_);
			return Event::Changed;
		case kPreviousTaps:
			pending_ = Previous(pending_);
			return Event::Changed;
		case kLeaveTaps:
			active_ = false;
			pending_ = running_;
			return Event::ModeLeft;
		default:
			return Event::None;   // the idle timer restarts from this series' last tap
		}
	}

	static Strategy Step(Strategy strategy, std::size_t forward)
	{
		constexpr std::size_t count = std::size(kStrategyOrder);
		for(std::size_t i = 0; i < count; i++)
		{
			if(kStrategyOrder[i] == strategy)
			{
				return kStrategyOrder[(i + forward) % count];
			}
		}
		return kStrategyOrder[0];
	}

	Strategy running_ = kDefaultStrategy;
	Strategy pending_ = kDefaultStrategy;
	bool active_ = false;
	uint8_t series_taps_ = 0;
	uint32_t last_tap_ms_ = 0;
};

#endif /* ROBOT_IS_MULTI_ALGORITHM */

#endif /* ALGORITHMS_INC_STRATEGYTAPSELECTOR_HPP_ */
