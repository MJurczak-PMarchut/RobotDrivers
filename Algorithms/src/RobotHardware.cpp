/*
 * RobotHardware.cpp
 *
 * Hardware objects and HAL callbacks shared by the MiniSumoV2 strategies; see
 * RobotHardware.hpp. Compiled only for the robots that use them.
 */
#include "stm32h7xx_hal.h"
#if defined(ROBOT_MT_V4) || defined(ROBOT_MT_V5) || defined(ROBOT_MINI_NEURO_V1) || defined(ROBOT_MINI_NEURO_V2) || \
	defined(ROBOT_ALGO_SELECT)
#include "RobotHardware.hpp"

extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim3;   // LEFT/RIGHT_MOTOR_TIMER_PTR (RobotSpecificDefines.hpp)
extern TIM_HandleTypeDef htim4;

L9960T MOTOR_CONTROLLERS[] = {
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, LEFT_MOTOR_PWM_CHANNEL, LEFT_MOTOR_TIMER_PTR, LEFT_MOTOR_INVERTED_PWM, true, false),
		[MOTOR_RIGHT] = L9960T(MOTOR_RIGHT, &hspi2, &MainCommManager, RIGHT_MOTOR_PWM_CHANNEL, RIGHT_MOTOR_TIMER_PTR, RIGHT_MOTOR_INVERTED_PWM, true, false)};

LSM6DSO IMU = LSM6DSO(&MainCommManager, &hspi2);

DirtyLogger logger = DirtyLogger(&retSD, SDPath, &SDFatFS, &SDFile);

volatile uint16_t SigPerSPADThreshold[std::size(Sensors)];

// Line sensors of this board: the digital configuration, LDLeft and LDRight at the front
// corners, about 7 mm behind the front edge and about 60 mm apart (user's estimate, as in
// SumoSimulation's RobotModel). The other configurations (2 or 4 analog sensors, a matrix) are
// other LineSensorArray classes with their own tables.
static const LineSensorMount kLineMounts[] = {
	{43, 30},    // LDLeft
	{43, -30},   // LDRight
};
static const DigitalLinePin kLinePins[] = {
	{LDLeft_GPIO_Port, LDLeft_Pin},
	{LDRight_GPIO_Port, LDRight_Pin},
};
static DigitalLineSensors DigitalLines(kLineMounts, kLinePins);
LineSensorArray& LineSensors = DigitalLines;

uint32_t DigitalLineSensors::Read(void)
{
	uint32_t on_line = 0;
	for(std::size_t i = 0; i < pins.size(); i++)
	{
		if(HAL_GPIO_ReadPin(pins[i].port, pins[i].pin) == GPIO_PIN_RESET)
		{
			on_line |= 1u << i;
		}
	}
	return on_line;
}

#ifdef DEBUG_SCREEN
// Qwiic/STEMMA QT SSD1327 module - I2C only (no reset line broken out on the
// Qwiic connector); OLED_I2C_ADDRESS (below) confirmed on this board's
// silkscreen/datasheet.
#define OLED_I2C_ADDRESS 0x3D
#define OLED_WIDTH 128
#define OLED_HEIGHT 128
// The OLED dirty-tracking grid (GRAYOLED_WINDOW_COLS x GRAYOLED_WINDOW_ROWS) is configured via
// build flags in CMakePresets.json (the presets' CUBE_CMAKE_CXX_FLAGS), NOT here: the
// SSD1327/GrayOLED driver is a separate translation unit and never sees #defines from this
// file. Default (no flags) is 1x1 = one whole-screen window; V4/V5/miniNeuro use 2x2 = 4 windows.

// No reset pin wired (Qwiic connector is power+I2C only) - matches Adafruit_SSD1327's
// own default of no reset pin for I2C-only wiring.
Adafruit_SSD1327 display = Adafruit_SSD1327(OLED_WIDTH, OLED_HEIGHT, &Wire);

// display.display() is flushed off the Robot task by DisplayThread; see DisplayMortalThread
// in RobotHardware.hpp for why the (100ms+) I2C flush is isolated onto its own task.
// RequestUpdate() is the non-blocking entry point other code should call.
DisplayMortalThread DisplayThread;

boundingBox StatsBox[4] = {boundingBox(0, "TOF"), boundingBox(1, "Gyro"),
	boundingBox(2, "Strat"), boundingBox(3, "misc")};

void _init_pg_draw_boundry(void)
{
	display.drawRect(0, 9, 128, 10, SSD1327_WHITE);
}

void _init_pg_draw_percentage(int16_t percentage)
{
	static int16_t drawn_bar = 0;
	int16_t width = (percentage * 1.24) - drawn_bar;
	display.fillRect(drawn_bar + 2, 11, width, 6, SSD1327_WHITE);
}

void _init_log_print_message(const char* str)
{
	static int16_t CurY = 20;
	display.setCursor(0, CurY);
	display.println(str);
	CurY = display.getCursorY();
}

void _init_display(void)
{
	if (!display.begin(OLED_I2C_ADDRESS))
	{
		logger.Log("--- Display begin() FAILED ---", LOGLEVEL_INFO);
	}

	DisplayThread.run();
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1327_WHITE, SSD1327_BLACK);
	display.setCursor(0, 0);
}

void _stat_draw_bounding_boxes()
{
	display.drawRect(0, 9, 128, 128-9, SSD1327_WHITE);
	display.drawFastVLine(63, 9, (128-9),SSD1327_WHITE);
	display.drawFastHLine(0, 63, 128,SSD1327_WHITE);
}
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == IMU_INT1_Pin || GPIO_Pin == IMU_INT2_Pin){
		IMU.InterruptCallback(GPIO_Pin);
		return;
	}
	// EXTI_Callback_func() matches the pin against every sensor in Sensors[] and ignores
	// any other pin (START, unused TOF_INT0/1).
	if(ToF_Sensor::CheckInitializationCplt()){
		ToF_Sensor::EXTI_Callback_func(GPIO_Pin);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
			MOTOR_CONTROLLERS[MOTOR_LEFT].SoftPWMCB_pulse();
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
			MOTOR_CONTROLLERS[MOTOR_RIGHT].SoftPWMCB_pulse();
	}
}

#endif
