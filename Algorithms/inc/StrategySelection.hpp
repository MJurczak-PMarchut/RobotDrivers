/*
 * StrategySelection.hpp
 *
 * Which strategy (algorithm) the multi-algorithm firmware runs, kept in non-volatile storage
 * so that it survives a reset and a reflash. Two independent stores:
 *  - Flash: records in the last sector of flash bank 2 (0x081E0000, kept out of the FLASH
 *    region in STM32H743IIKX_FLASH.ld). Survives everything but a full-chip erase and is
 *    built to survive a power cut during a write (see StrategySelection.cpp).
 *  - SdCard: the text file strategy.txt in the root directory of the card, holding one short
 *    name ("V4", "V5", "NEURO1", "NEURO2"), so it can be edited on a PC.
 * Which store wins at boot is the caller's policy (multiAlgorithmV1, step E).
 *
 * Only for the multi-algorithm firmware (ROBOT_IS_MULTI_ALGORITHM). Not reentrant: call
 * SetStrategy() / GetStrategy() from one task at a time.
 */

#ifndef ALGORITHMS_INC_STRATEGYSELECTION_HPP_
#define ALGORITHMS_INC_STRATEGYSELECTION_HPP_

#ifdef ROBOT_IS_MULTI_ALGORITHM

#include <cstdint>

// The strategies of the multi-algorithm firmware. The values are stored in flash: never
// renumber them, only add new ones.
enum class Strategy : uint8_t
{
	MiniTomiV4 = 1,
	MiniTomiV5 = 2,
	MiniNeuroV1 = 3,
	MiniNeuroV2 = 4,
};

// Runs when no store holds a valid strategy.
inline constexpr Strategy kDefaultStrategy = Strategy::MiniTomiV4;

// Order of the strategies when choosing one by taps (StrategyTapSelector: next / previous).
inline constexpr Strategy kStrategyOrder[] = {
	Strategy::MiniTomiV4,
	Strategy::MiniTomiV5,
	Strategy::MiniNeuroV1,
	Strategy::MiniNeuroV2,
};

enum class StrategyStorage : uint8_t
{
	Flash,
	SdCard,
};

// Stores `strategy` in `storage`. False if it could not be stored (unknown strategy, no card,
// flash error). Flash: storing the strategy that is already stored there writes nothing.
bool SetStrategy(Strategy strategy, StrategyStorage storage);

// Reads the strategy from `storage`. False if `storage` holds no valid strategy or is not
// available; `strategy` is then left unchanged. SdCard needs the RTOS scheduler running (the
// SD driver waits for its DMA on an RTOS queue); before that it returns false.
bool GetStrategy(Strategy& strategy, StrategyStorage storage);

// The strategy this firmware runs, as chosen at boot (DebugEntryCPP.cpp, ROBOT_ALGO_SELECT).
// kDefaultStrategy until SetRunningStrategy() is called.
void SetRunningStrategy(Strategy strategy);
Strategy GetRunningStrategy(void);

// Short name, as in strategy.txt and in logs: "V4", "V5", "NEURO1", "NEURO2" ("?" if unknown).
const char* StrategyName(Strategy strategy);
// Inverse of StrategyName(), ignoring letter case. False if `name` names no strategy.
bool StrategyFromName(const char* name, Strategy& strategy);

#endif /* ROBOT_IS_MULTI_ALGORITHM */

#endif /* ALGORITHMS_INC_STRATEGYSELECTION_HPP_ */
