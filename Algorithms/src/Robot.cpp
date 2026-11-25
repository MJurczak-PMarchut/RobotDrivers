/*
 * Robot.cpp
 *
 * Default implementations of the Robot base class (Algo.hpp). Compiled for every robot:
 * Robot::Calibrate() is the first non-inline virtual method of Robot, so the vtable of Robot
 * is emitted here.
 */
#include "stm32h7xx_hal.h"
#include "Algo.hpp"
#if defined(ROBOT_MT_V4) || defined(ROBOT_MT_V5) || defined(ROBOT_MINI_NEURO_V1) || defined(ROBOT_MINI_NEURO_V2) || \
	defined(ROBOT_ALGO_SELECT)
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

/*
 * Choosing the strategy by taps (multi-algorithm firmware, DEBUG_SCREEN only). The gesture
 * logic is StrategyTapSelector; here: the IMU's tap detector, the screen and storing the
 * choice. Design: ~/Projekty/MultiAlgorithmV1WorkState.md, "Wybór strategii puknięciami".
 */
#ifdef ROBOT_ALGO_SELECT
#ifdef DEBUG_SCREEN
#include "StrategySelection.hpp"
#include "StrategyTapSelector.hpp"

namespace {

// LSM6DSO single-tap detector, tuned on the robot in the V5 tap probe (2026-11-22): threshold
// 2 = 0.5 g on every axis (steps of 250 mg at +-8 g), shock 3 = 14.4 ms, quiet 3 = 7.2 ms,
// fed by the high-pass filter (not the slope filter, which missed taps).
constexpr uint8_t kTapThreshold = 2;
constexpr uint8_t kTapShock = 3;
constexpr uint8_t kTapQuiet = 3;
constexpr bool kTapHighPass = true;
// A tap sooner than this after the last counted one is the same knock ringing on (the robot
// rocking on its wheels): logged as "TAP dup", not counted.
constexpr uint32_t kTapDeadTimeMs = 120;
// Tap dots (bottom-left "misc" box) are cleared this long after the last tap.
constexpr uint32_t kDotsResetMs = 4000;

bool tap_detector_on = false;
StrategyTapSelector selector;
bool mode_shown = false;           // the selection screen is up
bool any_tap = false;
uint32_t last_tap_ms = 0;          // last counted tap
uint32_t tap_count = 0;

const unsigned char* GlyphFor(Strategy strategy)
{
	switch(strategy)
	{
	case Strategy::MiniTomiV4:
		return kGlyphMiniTomiV4;
	case Strategy::MiniTomiV5:
		return kGlyphMiniTomiV5;
	case Strategy::MiniNeuroV1:
		return kGlyphMiniNeuroV1;
	case Strategy::MiniNeuroV2:
		return kGlyphMiniNeuroV2;
	}
	return epd_bitmap_Untitled;
}

/*
 * Tap dots in the bottom-left box (StatsBox[3], in place of its glyph): one dot per counted
 * tap, the dots of one series next to each other, a wider gap before each new series; rows
 * wrap, a full box starts over. kDotsResetMs after the last tap the box gets its glyph back.
 */
constexpr int16_t kDotRadius = 2;
constexpr int16_t kDotSize = 2 * kDotRadius + 1;
constexpr int16_t kDotStep = kDotSize + 2;        // dots of one series
constexpr int16_t kSeriesExtraGap = 2 * kDotSize; // added before a new series
constexpr int16_t kDotRowStep = kDotSize + 4;
int16_t dot_x = 0;   // offset of the next dot in the content area
int16_t dot_y = 0;
bool dots_shown = false;

void DotsClear(void)
{
	StatsBox[3].clear_content();
	StatsBox[3].draw_glyph();
	dot_x = 0;
	dot_y = 0;
	dots_shown = false;
}

void DotsAdd(bool new_series)
{
	if(!dots_shown)
	{
		StatsBox[3].clear_content();
	}
	else if(new_series)
	{
		dot_x += kSeriesExtraGap;
	}
	if(dot_x + kDotSize > StatsBox[3].content_width())
	{
		dot_x = 0;
		dot_y += kDotRowStep;
	}
	if(dot_y + kDotSize > StatsBox[3].content_height())
	{
		StatsBox[3].clear_content();
		dot_x = 0;
		dot_y = 0;
	}
	display.fillCircle(StatsBox[3].content_x() + dot_x + kDotRadius,
			StatsBox[3].content_y() + dot_y + kDotRadius, kDotRadius, SSD1327_WHITE);
	dot_x += kDotStep;
	dots_shown = true;
}

// The WAIT_FOR_START screen as the strategies draw it after calibration.
void ShowWaitScreen(void)
{
	StatsBox[1].clear_content();
	StatsBox[1].print_wrapped("AAAAA!!! Napierdalać!!!");
	StatsBox[2].clear_content();
	StatsBox[2].draw_glyph(GlyphFor(GetRunningStrategy()));
	mode_shown = false;
}

// Selection mode: the chosen strategy's glyph in "Strat", its name and the gesture in box 1
// ("*" marks the running strategy).
void ShowSelectionScreen(void)
{
	char text[48];
	const Strategy pending = selector.Pending();
	std::snprintf(text, sizeof(text), "WYBOR:\n%s%s\n1:+ 2:-\n3:wyjdz", StrategyName(pending),
			(pending == selector.Running())? " *" : "");
	StatsBox[1].clear_content();
	StatsBox[1].print_wrapped(text);
	StatsBox[2].clear_content();
	StatsBox[2].draw_glyph(GlyphFor(pending));
	mode_shown = true;
}

// Stores the chosen strategy (flash, and the card if strategy.txt is there: the card wins at
// boot) and restarts. Returns only if nothing could be stored.
void CommitAndRestart(Strategy strategy)
{
	MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
	char text[48];
	std::snprintf(text, sizeof(text), "Zapis:\n%s\nrestart", StrategyName(strategy));
	StatsBox[1].clear_content();
	StatsBox[1].print_wrapped(text);
	DisplayThread.RequestUpdate();
	vTaskDelay(pdMS_TO_TICKS(200));   // let DisplayThread show it before the flash write
	
	const bool flash_ok = SetStrategy(strategy, StrategyStorage::Flash);
	Strategy on_card;
	const bool card_file = GetStrategy(on_card, StrategyStorage::SdCard);
	const bool card_ok = card_file && SetStrategy(strategy, StrategyStorage::SdCard);
	char message[100];
	std::snprintf(message, sizeof(message), "Strategy %s selected by taps (flash %s, SD %s)",
			StrategyName(strategy), flash_ok ? "OK" : "FAILED",
			card_file ? (card_ok ? "OK" : "FAILED") : "none");
	logger.Log(message, LOGLEVEL_INFO);
	if(!flash_ok && !card_ok)
	{
		StatsBox[1].clear_content();
		StatsBox[1].print_wrapped("Zapis\nFAILED");
		DisplayThread.RequestUpdate();
		return;
	}
	// Flush(), not Sync(): the staged sub-sector tail has to reach the card before the reset.
	logger.Flush();

	HAL_GPIO_WritePin(EXT_LDO_EN_GPIO_Port, EXT_LDO_EN_Pin, GPIO_PIN_RESET);
	NVIC_SystemReset();
}

} // namespace

void Robot::StrategySelectionReset(void)
{
	if(!tap_detector_on)
	{
		const HAL_StatusTypeDef status = IMU.EnableTapDetection(kTapThreshold, kTapThreshold,
				kTapThreshold, kTapShock, kTapQuiet, kTapHighPass);
		tap_detector_on = (status == HAL_OK);
		logger.Log(tap_detector_on ? "Strategy selection by taps: on" :
				"Strategy selection by taps: tap detector FAILED", LOGLEVEL_INFO);
	}
	// Taps while fighting do not count: drop one latched since the last WAIT_FOR_START.
	LSM6DSOTapEvent unused;
	(void)IMU.ReadTap(unused);
	selector.Reset(GetRunningStrategy());
	any_tap = false;
	if(mode_shown)   // left WAIT_FOR_START (start signal) while choosing: choice dropped
	{
		ShowWaitScreen();
	}
	DotsClear();
	DisplayThread.RequestUpdate();
}

void Robot::StrategySelectionStep(void)
{
	if(!tap_detector_on)
	{
		return;
	}
	const uint32_t now = HAL_GetTick();
	bool redraw = false;
	uint8_t taps = 0;
	LSM6DSOTapEvent tap;
	if(IMU.ReadTap(tap))
	{
		const uint32_t dt_ms = now - last_tap_ms;
		char message[64];
		if(any_tap && (dt_ms < kTapDeadTimeMs))
		{
			std::snprintf(message, sizeof(message), "TAP dup dt=%lums", static_cast<unsigned long>(dt_ms));
		}
		else
		{
			const bool new_series = !any_tap || (dt_ms >= StrategyTapSelector::kSeriesGapMs);
			any_tap = true;
			last_tap_ms = now;
			tap_count++;
			taps = 1;
			DotsAdd(new_series);
			redraw = true;
			std::snprintf(message, sizeof(message), "TAP #%lu dt=%lums", static_cast<unsigned long>(tap_count),
					static_cast<unsigned long>(dt_ms));
		}
		logger.Log(message, LOGLEVEL_INFO);
	}
	else if(dots_shown && ((now - last_tap_ms) >= kDotsResetMs))
	{
		DotsClear();
		redraw = true;
	}

	char message[64];
	switch(selector.Update(now, taps))
	{
	case StrategyTapSelector::Event::None:
		break;
	case StrategyTapSelector::Event::ModeEntered:
		logger.Log("Strategy selection: mode on", LOGLEVEL_INFO);
		ShowSelectionScreen();
		redraw = true;
		break;
	case StrategyTapSelector::Event::Changed:
		std::snprintf(message, sizeof(message), "Strategy selection: %s", StrategyName(selector.Pending()));
		logger.Log(message, LOGLEVEL_INFO);
		ShowSelectionScreen();
		redraw = true;
		break;
	case StrategyTapSelector::Event::ModeLeft:
		logger.Log("Strategy selection: mode off, no change", LOGLEVEL_INFO);
		ShowWaitScreen();
		redraw = true;
		break;
	case StrategyTapSelector::Event::Commit:
		CommitAndRestart(selector.Pending());
		// Only if nothing could be stored: the running strategy stays.
		selector.Reset(GetRunningStrategy());
		vTaskDelay(pdMS_TO_TICKS(1500));
		ShowWaitScreen();
		redraw = true;
		break;
	}
	if(redraw)
	{
		DisplayThread.RequestUpdate();
	}
}

#else  /* !DEBUG_SCREEN: no screen, no selection by taps */
void Robot::StrategySelectionReset(void) {}
void Robot::StrategySelectionStep(void) {}
#endif /* DEBUG_SCREEN */
#endif /* ROBOT_ALGO_SELECT */
