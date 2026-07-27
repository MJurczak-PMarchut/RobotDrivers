/*
 * MiniTomiV4Helpers.hpp
 *
 * Display/statistics drawing helpers, the heading-PID relay auto-tuner, and the
 * movement sub-routines that used to live inline in miniTomiV4.cpp. Splitting them
 * out keeps the V4 strategy file focused on Robot::begin/loop and the match FSM.
 *
 * This is an *internal* header shared only by the ROBOT_MT_V4 translation units
 * (miniTomiV4.cpp + MiniTomiV4Helpers.cpp): it declares, with external linkage, the
 * file-scope objects those two units share (display, logger, IMU, ...) whose single
 * definitions live in miniTomiV4.cpp. Include it only from a source guarded by
 * #ifdef ROBOT_MT_V4.
 */

#ifndef ALGORITHMS_INC_MINITOMIV4HELPERS_HPP_
#define ALGORITHMS_INC_MINITOMIV4HELPERS_HPP_

#include "cmsis_os2.h"
#include "osapi.h"
#include "osapi_mortal_thread.h"
#include "semphr.h"
#include "Adafruit_SSD1327.h"
#include "DirtyLogger.hpp"
#include "lsm6dso.hpp"
#include "vl53l1x.hpp"
#include "L9960T.hpp"
#include "HeadingPID.hpp"
#include "Algo.hpp"
#include <cstring>


const unsigned char epd_bitmap_Untitled [] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0e, 0xd8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xcc, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x70, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x06, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x3e, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0xce, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x20, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x14, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x20, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x04, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x60, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x04, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x60, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x04, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x20, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0c, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x20, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0c, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x20, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0c, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x20, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x08, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x20, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 0x01, 0x20, 0x08, 0x7c, 0x00, 0x00, 0x00, 
	0x00, 0x1f, 0xf0, 0x09, 0xce, 0x00, 0x00, 0x00, 0x00, 0x18, 0xf0, 0x0f, 0x06, 0x00, 0x00, 0x00, 
	0x00, 0x10, 0x70, 0x1c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x18, 0x03, 0x00, 0x00, 0x00, 
	0x00, 0x30, 0x38, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x20, 0x18, 0x00, 0x02, 0x00, 0x00, 0x00, 
	0x00, 0x60, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 
	0x00, 0x60, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 
	0x00, 0x18, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x06, 0x00, 0x0c, 0x00, 0x00, 0x00, 
	0x00, 0x07, 0xff, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf1, 0x80, 0x30, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0xc1, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00
};






// --- V4 tuning constants shared between the strategy file and its helpers ---
#define OLED_I2C_ADDRESS 0x3D
#define DETECTION_DISTANCE 350
#define FULL_SPEED 0.85
#define MANOUVER_SPEED 0.65
#define ROTATION_TIMEOUT 150
#define MAX_SIG_PER_SPAD	8000

// Errors smaller than this are treated as zero, so sensor noise / motor response lag
// doesn't make the robot hunt back and forth for a negligible heading difference.
#define HEADING_PID_DEADBAND_DEG 15.0f

// Match FSM states: the strategy file (miniTomiV4.cpp) drives the machine, while
// LookForOpponent2() returns the next state to enter.
typedef enum{CALIBRATE, WAIT_FOR_START, FIGHT, OUT_OF_BOUNDS, LOOK_FOR_OPPONENT_1, LOOK_FOR_OPPONENT_2, NOP} State_t;

typedef struct{
	uint16_t raw_left;
	uint16_t raw_right;
	uint16_t left;
	uint16_t right;
	uint16_t raw_rear;
	// SigPerSPAD behind each front reading - the quantity GetValidatedDistances()
	// actually gates on (vs. maxVal_perSPAD[]), carried out so callers can log the
	// reason a range was accepted or replaced instead of only the outcome.
	uint16_t sig_left;
	uint16_t sig_right;
}SensorDistances_t;

/*
 * File-scope objects defined in miniTomiV4.cpp, shared with the helpers TU.
 */
extern Adafruit_SSD1327 display;
extern DirtyLogger logger;
extern LSM6DSO IMU;
extern HeadingPID HeadingController;
extern L9960T MOTOR_CONTROLLERS[];
extern VL53L1X Sensors[];
extern volatile uint16_t maxVal_perSPAD[2];
extern volatile bool lastDetOnLeft;

// Which line sensor(s) fired on the most recent out-of-bounds event; latched by
// isOutOfBounds() (defined in the helpers TU) and read back by the OOB handler in
// Robot::loop(). Defined in MiniTomiV4Helpers.cpp.
extern volatile bool lineDetOnLeft;
extern volatile bool lineDetOnRight;

// Adafruit_SSD1327::display() flushes the framebuffer one 64-byte row at a time (128
// rows for a full 128x128 frame), and each row's I2C write blocks the calling task
// until that chunk completes (see Adafruit_I2CDevice::write() in ArduinoCompat) - a
// full-frame update can take on the order of 100ms+. Running it on its own idle-priority
// task keeps that off the Robot task (and everything else), matching how
// ToF_Sensor::StartSensorTask() already isolates sensor polling onto its own task
// (RobotDrivers/ToF_sensors/Common/ToFSensor.cpp). RequestUpdate() is the non-blocking
// entry point other code should call instead of `display.display()` directly.
class DisplayMortalThread : public MortalThread
{
public:
	DisplayMortalThread() : MortalThread(tskIDLE_PRIORITY, 1024, "DisplayThread")
	{
		updateSemaphore = xSemaphoreCreateBinary();
		updateCompleteSemaphore = xSemaphoreCreateBinary();
	}

	void RequestUpdate(void)
	{
		xSemaphoreGive(updateSemaphore);
	}

	// Blocks until the display() triggered by the next RequestUpdate() has actually
	// returned (all rows genuinely transferred or honestly failed - see
	// Adafruit_I2CDevice::write()), not just until DisplayThread has started running.
	// Only meant for the one-shot startup draw in Robot::begin(): waiting here for
	// real completion, instead of a fixed delay or an isInitCompleted() check, is
	// what reliably avoids racing the boot-time draw against Robot::loop() starting.
	void WaitForUpdateComplete(void)
	{
		xSemaphoreTake(updateCompleteSemaphore, portMAX_DELAY);
	}

protected:
	void loop(void) override
	{
		if (xSemaphoreTake(updateSemaphore, portMAX_DELAY) == pdTRUE)
		{
			display.display();
			xSemaphoreGive(updateCompleteSemaphore);
		}
	}

private:
	SemaphoreHandle_t updateSemaphore;
	SemaphoreHandle_t updateCompleteSemaphore;
};

extern DisplayMortalThread DisplayThread;

class boundingBox{
	public:
		boundingBox(uint8_t quadrant, const char* title) : _title{title} {
			_y = (quadrant == 0 || quadrant == 1)? 10 : 64;
			_x = (quadrant == 1 || quadrant == 2)? 64 : 1;
			// Bottom of this box's drawable area: top boxes stop just above the y=63 divider,
			// bottom boxes just above the y=127 outer border. Used to clip print_wrapped().
			_yBottom = (quadrant == 0 || quadrant == 1)? 63 : 127;
			stat_pointer = 0;
		}
		void attach_stat(const char* stat)
		{
			if(stat_pointer < 4){ // must match the _stat[4] array size below
				_stat[stat_pointer] = stat;
				stat_pointer++;
			}
		}
		void printstat(const char* str, const char* stat)
		{
			uint8_t stat_iter = 0;
			for(stat_iter = 0; stat_iter < 4; stat_iter++)
			{
				if(_stat[stat_iter] != nullptr && strcmp(stat, _stat[stat_iter]) == 0){
					display.setCursor(_x + 1, stat_iter * 9 + _y + 11);
					display.print(str);
					break;
				}
			}

		}
		void draw_all(void){
			_display_title();
		}
		void draw_glyph()
		{
			display.drawBitmap(_x + 1, _y + 10, epd_bitmap_Untitled, 61, 49, SSD1327_WHITE);
		}
		// Print `str` inside this box only: word-wrap at the box's right edge (returning to the
		// box's left, not the screen's) and clip at the box bottom, so text never bleeds into a
		// neighbouring quadrant the way display.print() would (GFX wraps at the full panel width
		// and treats '\n' as return-to-x=0). A word wider than one line falls back to char-wrap.
		// Assumes the classic 6x8 font at text size 1 (6px advance, 9px line pitch - the metrics
		// printstat()/the box frame already use). startLine offsets the first line down within the
		// box (0 = just under the title bar). Global text-wrap is left untouched: every glyph is
		// placed explicitly and each word is pre-checked to fit, so GFX's own wrap never fires.
		void print_wrapped(const char* str, uint8_t startLine = 0)
		{
			const int16_t charW = 6, lineH = 9;
			const int16_t left = _x + 1, right = _x + 62, bottom = _yBottom;
			int16_t cx = left;
			int16_t cy = _y + 11 + startLine * lineH;
			if (cy + 8 > bottom) return;

			while (*str)
			{
				if (*str == '\r') { str++; continue; }
				if (*str == '\n')
				{
					cx = left; cy += lineH; str++;
					if (cy + 8 > bottom) return;
					continue;
				}
				// Measure the next whitespace-delimited word.
				const char* end = str;
				while (*end && *end != ' ' && *end != '\n') end++;
				int16_t wordW = (int16_t)(end - str) * charW;
				// Move a word that will not fit down to the next line (unless already at line start).
				if (cx > left && cx + wordW > right) { cx = left; cy += lineH; }
				if (cy + 8 > bottom) return;
				// Draw the word, char-wrapping if it is wider than a whole line.
				for (const char* p = str; p < end; p++)
				{
					if (cx + charW > right)
					{
						cx = left; cy += lineH;
						if (cy + 8 > bottom) return;
					}
					display.setCursor(cx, cy);
					display.write((uint8_t)*p);
					cx += charW;
				}
				str = end;
				// Consume one space separator: draw it if it fits on the line, else wrap.
				if (*str == ' ')
				{
					str++;
					if (cx + charW <= right)
					{
						display.setCursor(cx, cy);
						display.write((uint8_t)' ');
						cx += charW;
					}
					else { cx = left; cy += lineH; if (cy + 8 > bottom) return; }
				}
			}
		}
	private:
		void _display_title(void){
			display.drawFastHLine(_x, _y + 9, 63, SSD1327_WHITE);
			display.setCursor(_x+1, _y+1);
			display.print(_title);
		}
		const char* _title;
		int16_t _y;
		int16_t _x;
		int16_t _yBottom; ///< bottom y of this box's drawable area (clip line for print_wrapped)
		const char* _stat[4] = {nullptr};
		uint8_t stat_pointer;
};

extern boundingBox StatsBox[4];

/*
 * Boot/statistics display drawing (implemented in MiniTomiV4Helpers.cpp).
 */
void _init_pg_draw_boundry(void);
void _init_pg_draw_percentage(int16_t percentage);
void _init_log_print_message(const char* str);
void _init_display(void);
void _stat_draw_bounding_boxes(void);

/*
 * Sensing / FSM helpers (implemented in MiniTomiV4Helpers.cpp), called from the
 * strategy file's Robot::loop().
 */
SensorDistances_t GetValidatedDistances(void);
bool isOutOfBounds(void);
bool isOpponentDetected(void);
const char* StateName(State_t state);
State_t CheckStartCondition(Robot *obj, State_t eFSM_state);

/*
 * Calibration and movement sub-routines (implemented in MiniTomiV4Helpers.cpp).
 */
void PID_CalibrateHeading(uint32_t maxDurationMs, float relayPower);
void FightAlgorithm(Robot *obj);
State_t LookForOpponent2(Robot *obj);
void Calibrate(void);

#endif /* ALGORITHMS_INC_MINITOMIV4HELPERS_HPP_ */
