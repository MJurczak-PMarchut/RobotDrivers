/*
 * RobotHardware.hpp
 *
 * Hardware objects and HAL callbacks shared by the MiniSumoV2 strategies (V4, V5,
 * miniNeuroV1, miniNeuroV2): motors, IMU, ToF sensors, SD logger, OLED display, its
 * flush task and the debug-screen drawing (DEBUG_SCREEN). There is one instance of each per robot, whichever algorithm runs, so they
 * live here instead of in the strategy files. Defined in RobotHardware.cpp, except
 * Sensors (see below).
 *
 * Include it only from a source compiled for one of those robots (ROBOT_MT_V4,
 * ROBOT_MT_V5, ROBOT_MINI_NEURO_V1, ROBOT_MINI_NEURO_V2).
 */

#ifndef ALGORITHMS_INC_ROBOTHARDWARE_HPP_
#define ALGORITHMS_INC_ROBOTHARDWARE_HPP_

#include <cstring>
#include <iterator>
#include <span>
#include "osapi.h"
#include "semphr.h"
#include "Adafruit_SSD1327.h"
#include "CommManager.hpp"
#include "DirtyLogger.hpp"      // also brings retSD/SDPath/SDFatFS/SDFile (fatfs.h)
#include "L9960T.hpp"
#include "lsm6dso.hpp"
#include "vl53l1x.hpp"

extern I2C_HandleTypeDef hi2c1;
extern CommManager MainCommManager;

extern L9960T MOTOR_CONTROLLERS[];   // indexed by MOTOR_LEFT / MOTOR_RIGHT
extern LSM6DSO IMU;
extern DirtyLogger logger;

// Inline variable (C++17) so that every translation unit sees the full type VL53L1X[N]:
// std::size(Sensors) works everywhere and N follows this initializer list.
// The order is the physical ToF slot: the VL53L1X constructor numbers the sensors in
// construction order, and that number selects the pins from XSHUT_PINS / TOFx_GPIO_PINS.
inline VL53L1X Sensors[] = {
		VL53L1X(FRONT_LEFT, &MainCommManager, &hi2c1),
		VL53L1X(BACK, &MainCommManager, &hi2c1),
		VL53L1X(FRONT_RIGHT, &MainCommManager, &hi2c1)
};

// SigPerSPAD threshold of each ToF sensor, indexed like Sensors[]: a reading counts only when
// its signal is above it. Set by the default Robot::Calibrate() (Robot.cpp); the strategies
// that use it (MiniTomiV4, MiniTomiV5) load the pre-calibration value in begin().
extern volatile uint16_t SigPerSPADThreshold[std::size(Sensors)];

// Line (edge) sensors. The board is reconfigurable: 2 digital (comparator) sensors, 2 or 4
// analog ones, or a sensor matrix. Whatever their kind, every sensor ends up as one bool per
// point, "this point sees the line", so the strategies only use LineSensors: a mask from Read()
// (bit i = sensor i, up to 32) and where each sensor sits (Mount(i)).

// Mounting point of one line sensor relative to the robot centre, in mm: x forward, y to the
// left (as LineSensorMount in SumoSimulation).
struct LineSensorMount
{
	int16_t x_mm;
	int16_t y_mm;
};

class LineSensorArray
{
public:
	// Bit i set = sensor i (Mount(i)) sees the line now.
	virtual uint32_t Read(void) = 0;
	std::size_t Count(void) const { return mounts.size(); }
	const LineSensorMount& Mount(std::size_t i) const { return mounts[i]; }
protected:
	explicit LineSensorArray(std::span<const LineSensorMount> sensorMounts) : mounts(sensorMounts) {}
	~LineSensorArray() = default;   // never deleted through the base
private:
	std::span<const LineSensorMount> mounts;
};

// Digital configuration: one comparator output per sensor on a GPIO, active low (low = line).
struct DigitalLinePin
{
	GPIO_TypeDef* port;
	uint16_t pin;
};

class DigitalLineSensors final : public LineSensorArray
{
public:
	// pins[i] belongs to mounts[i]: both arrays have N elements.
	template <std::size_t N>
	DigitalLineSensors(const LineSensorMount (&sensorMounts)[N], const DigitalLinePin (&sensorPins)[N])
		: LineSensorArray(sensorMounts), pins(sensorPins)
	{
		static_assert(N <= 32, "Read() returns one bit per sensor in 32 bits");
	}
	uint32_t Read(void) override;
private:
	std::span<const DigitalLinePin> pins;
};

// The line sensors of this board (RobotHardware.cpp).
extern LineSensorArray& LineSensors;

#ifdef DEBUG_SCREEN
// OLED (Qwiic SSD1327) and everything drawn on it exist only in DEBUG_SCREEN builds.
extern Adafruit_SSD1327 display;

// Only in DEBUG_SCREEN builds (the only ones that define and use DisplayThread). Without the
// guard, a file that sees this class as the only MortalThread with a loop() (e.g. Robot.cpp,
// which emits MortalThread::job() with the Robot vtable) lets GCC speculatively devirtualise
// job() into DisplayMortalThread::loop(), which references `display` - undefined there.
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

// 61x49 glyph drawn by boundingBox::draw_glyph().
inline const unsigned char epd_bitmap_Untitled[] = {
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

// One quadrant of the statistics screen drawn after calibration: a title bar and up to four
// named stats (attach_stat() / printstat()).
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
			if(stat_pointer < std::size(_stat)){
				_stat[stat_pointer] = stat;
				stat_pointer++;
			}
		}
		void printstat(const char* str, const char* stat)
		{
			uint8_t stat_iter = 0;
			for(stat_iter = 0; stat_iter < std::size(_stat); stat_iter++)
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

// The four statistics boxes (TOF, Gyro, Strat, misc), one per screen quadrant.
extern boundingBox StatsBox[4];

// Boot screen (progress bar + log lines) and the statistics screen frame.
void _init_pg_draw_boundry(void);
void _init_pg_draw_percentage(int16_t percentage);
void _init_log_print_message(const char* str);
void _init_display(void);
void _stat_draw_bounding_boxes(void);
#endif

#endif /* ALGORITHMS_INC_ROBOTHARDWARE_HPP_ */
