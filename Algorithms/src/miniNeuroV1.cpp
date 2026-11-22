/*
 * miniNeuroV1.cpp
 *
 * miniNeuroV1 strategy: copy of miniTomiV4.cpp (RobotDrivers 7db6ebe) whose tuning
 * constants are run-time parameters (MiniNeuroV1Params.hpp), tuned in SumoSimulation.
 * With the default parameters it behaves exactly like miniTomiV4. Differences in code:
 *  - every motor power has its own parameter (V4 shared FULL_SPEED / MANOUVER_SPEED);
 *  - the OUT_OF_BOUNDS turn can go away from the line sensor that fired (oob_turn_mode);
 *  - the heading PID gains are applied from the parameters in begin();
 *  - Calibrate() computes the SigPerSPAD threshold without the uint16_t wrap-around.
 *
 *  Based on miniTomi.cpp, created on: 12 gru 2022, author: Mateusz
 */
#include "cmsis_os2.h"
#include "osapi.h"
#include "portmacro.h"
#include "stm32h7xx_hal.h"
#include <cmath>
#include <stdio.h>
#include <string.h>
#if defined(ROBOT_MINI_NEURO_V1) || defined(ROBOT_ALGO_SELECT)   // ROBOT_ALGO_SELECT: all strategies in one image
#include "Algo.hpp"
#include "HeadingPID.hpp"
#include "vl53l1x.hpp"
#include "L9960T.hpp"
#include "DirtyLogger.hpp"
#include <string>
#include "lsm6dso.hpp"
#include "Adafruit_SSD1327.h"
#include "osapi_mortal_thread.h"
#include "semphr.h"
// Shared helpers/objects split out of this file (PID_CalibrateHeading,
// FightAlgorithm, LookForOpponent2, Calibrate, and the
// file-scope objects those share). Also maps the tuning names used below
// (DETECTION_DISTANCE, ROTATION_TIMEOUT, MAX_SIG_PER_SPAD...) onto NeuroParams.
#include "MiniNeuroV1Helpers.hpp"
using namespace neuro_v1;   // State_t, StateName, lastDetOnLeft, ... (MiniNeuroV1Helpers.hpp)

#define INIT_TIMEOUT_MS  6000


#define SEARCH_STRATEGY_CHANGE_TIME  ((TickType_t)NeuroParams.search_strategy_change_ms)

// Robot::loop() runs at least this often even without sensor/line events.
#define LOOP_WAKE_TIMEOUT_MS  10
// Above ToF/display/logger tasks, so an event wakes the FSM at once. Set after begin():
// init busy-waits (HAL_Delay, taskYIELD loops) and must not starve those tasks.
#define ROBOT_RUN_PRIORITY  (tskIDLE_PRIORITY + 2)

// Out-of-bounds escape: stop, back off, turn towards the last detection. Blind to the
// opponent until the end.
#define OOB_STOP_MS  ((uint32_t)NeuroParams.oob_stop_ms)
#define OOB_BACK_MS  ((uint32_t)NeuroParams.oob_back_ms)
#define OOB_TURN_MS  ((uint32_t)NeuroParams.oob_turn_ms)
enum {OOB_STOP = MANOEUVRE_IDLE + 1, OOB_BACK, OOB_TURN};

// The tuning parameters themselves live in MiniNeuroV1Params.hpp; MiniNeuroV1Helpers.hpp
// maps the V4 names onto them (shared with the helpers TU).

#define LED2_ON HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_OFF HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)

// Hardware objects (MOTOR_CONTROLLERS, IMU, Sensors, logger, display, DisplayThread):
// RobotHardware.{hpp,cpp}.


volatile bool neuro_v1::lastDetOnLeft = false;

// Display drawing helpers (_init_*/_stat_draw_bounding_boxes), the boundingBox class and
// StatsBox are shared by all strategies: RobotHardware.{hpp,cpp}, DEBUG_SCREEN only.

/*
 * Algorithm
 */
// OUT_OF_BOUNDS escape turn: towards the last detection (V4), or with oob_turn_mode = 1
// away from the line sensor that fired when only one of them did.
bool MiniNeuroV1::OobTurnLeft(void) const
{
	if(NeuroParams.oob_turn_mode >= 0.5 && (LineDetOnLeft() != LineDetOnRight()))
	{
		return LineDetOnRight();
	}
	return lastDetOnLeft;
}

void MiniNeuroV1::begin(void)
{
	// SigPerSPAD thresholds until Calibrate() sets them (RobotHardware.hpp).
	for (auto& threshold : SigPerSPADThreshold){
		threshold = 2000;
	}
	// Gains from the parameters (Robot's constructor used its default constants, Algo.hpp).
	heading_pid.SetGains((float)NeuroParams.pid_kp, (float)NeuroParams.pid_ki, (float)NeuroParams.pid_kd);
	was_running = true;
	flash_period_ms = 100;
	// HAL_StatusTypeDef imuret;
	logger.Init();
	logger.StartTimestamp();
	//Enable power
	//Enable power
	logger.Log("--- INIT Started ---", LOGLEVEL_INFO);

	HAL_GPIO_WritePin(EXT_LDO_EN_GPIO_Port, EXT_LDO_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NCS1_GPIO_Port, NCS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NCS2_GPIO_Port, NCS2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

	HAL_Delay(100);
	HAL_GPIO_WritePin(EXT_LDO_EN_GPIO_Port, EXT_LDO_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(250);
#ifdef DEBUG_SCREEN
	logger.Log("--- Display Started ---", LOGLEVEL_INFO);

	_init_display();
	display.println("MiniSumo Init Started");
	_init_pg_draw_boundry();
	_init_pg_draw_percentage(10);
	display.setCursor(0, 20);
	_init_log_print_message("---------------------");
	_init_log_print_message("SD Initialized");
	DisplayThread.RequestUpdate();
#endif
	IMU.Init();
	HAL_Delay(100);
	IMU.StartCalibrationOrientation();
#ifdef DEBUG_SCREEN
	_init_pg_draw_percentage(20);
	_init_log_print_message("IMU Initialized");
	DisplayThread.RequestUpdate();
#endif
	HAL_Delay(250);
#ifdef DEBUG_SCREEN
	_init_pg_draw_percentage(45);
#endif
	IMU.CalibrateOrientation();
#ifdef DEBUG_SCREEN
	_init_log_print_message("IMU Calibration");
	DisplayThread.RequestUpdate();
#endif
	HAL_Delay(200);
#ifdef DEBUG_SCREEN
	_init_log_print_message("Start ToF task");
	_init_pg_draw_percentage(75);
	DisplayThread.RequestUpdate();
#endif
	ToF_Sensor::StartSensorTask();
	logger.Log("--- Sensor Task Started---", LOGLEVEL_INFO);
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
	MOTOR_CONTROLLERS[MOTOR_LEFT].Init(0);
	logger.Log("--- Motors Started ---", LOGLEVEL_INFO);

	while(MOTOR_CONTROLLERS[MOTOR_LEFT].CheckIfControllerInitializedOk() != HAL_OK)
	{taskYIELD();}

	MOTOR_CONTROLLERS[MOTOR_RIGHT].Init(0);
	flash_period_ms = 300;
	while(MOTOR_CONTROLLERS[MOTOR_RIGHT].CheckIfControllerInitializedOk() != HAL_OK)
	{taskYIELD();}
	MCInterface::run();
	
	flash_period_ms = 2000;
	while(ToF_Sensor::CheckInitializationCplt() != true)
	{taskYIELD();}
#ifdef DEBUG_SCREEN
	_init_log_print_message("ToF OK");
	_init_pg_draw_percentage(100);
#endif

	logger.Log("--- INIT Completed ---", LOGLEVEL_INFO);
	
	MOTOR_CONTROLLERS[MOTOR_LEFT].Disable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Disable();
#ifdef DEBUG_SCREEN
	_init_log_print_message("INIT COMPLETED");
	_init_log_print_message("---------------------");
	DisplayThread.RequestUpdate();
	vTaskDelay(10);

	StatsBox[0].attach_stat("L");
	StatsBox[0].attach_stat("P");
	StatsBox[0].attach_stat("T");
#endif
	this->setPriority(ROBOT_RUN_PRIORITY);
}



// neuro_v1::PID_CalibrateHeading() (heading-PID relay auto-tune) lives in MiniNeuroV1Helpers.cpp;
// call it as neuro_v1::PID_CalibrateHeading(heading_pid, ...).

// GetValidatedDistances(), isOpponentDetected(), StateName() and
// CheckStartCondition() (plus the
// FightAlgorithm/LookForOpponent2/Calibrate sub-routines) now live in
// MiniTomiV4Helpers.cpp.

void MiniNeuroV1::loop(void)
{
	static State_t eFSM_state = CALIBRATE;
	static bool direction_ack = false;
	static TickType_t last_detection_tick = 0;
	static bool opponent_detected = false;
	static Manoeuvre manoeuvre;
	static State_t stepped_state = NOP;   // state handled by the previous switch pass
	static bool run_again = false;

	// Single wait point: any ToF sample, a new line detection, the manoeuvre phase
	// deadline or the timeout. A state change in the previous pass skips the wait.
	TickType_t wait_ticks = run_again ? 0 :
			manoeuvre.WaitTicks(pdMS_TO_TICKS(LOOP_WAKE_TIMEOUT_MS));
	EventBits_t events = 0;
	EventGroupHandle_t tof_events = ToF_Sensor::GetEventHandle();
	if(tof_events != NULL)
	{
		events = xEventGroupWaitBits(tof_events, TOF_EVENT_MASK | LINE_EVENT_BIT,
				pdTRUE, pdFALSE, wait_ticks);
	}
	else
	{
		vTaskDelay(pdMS_TO_TICKS(LOOP_WAKE_TIMEOUT_MS));
	}
	// Re-evaluate the opponent on every front-sensor sample: detection needs only one side,
	// and the other side's reading is at most one period (~20 ms) old. Waiting for both
	// added up to ~20 ms to LOOK -> FIGHT. TOF_EVAL_TIMEOUT_MS covers dead sensors.
	static TickType_t last_eval_tick = 0;
	TickType_t now = xTaskGetTickCount();
	bool new_tof_data = ((events & TOF_FRONT_MASK) != 0) ||
			((now - last_eval_tick) >= pdMS_TO_TICKS(TOF_EVAL_TIMEOUT_MS));
	if(new_tof_data)
	{
		last_eval_tick = now;
		opponent_detected = isOpponentDetected();
	}

	eFSM_state = CheckStartCondition(eFSM_state);
	// Edge before any state sets the motors. LOOK_FOR_OPPONENT_2 handles it itself.
	if((eFSM_state == FIGHT || eFSM_state == LOOK_FOR_OPPONENT_1) && isOutOfBounds())
	{
		eFSM_state = OUT_OF_BOUNDS;
	}
	// Collision handling: the IMU driver latches sudden accel/yaw-rate jumps (impacts)
	// that our own motor-driven moves don't produce. Deferred until a manoeuvre ends.
	if((eFSM_state == LOOK_FOR_OPPONENT_1 || eFSM_state == LOOK_FOR_OPPONENT_2) &&
			!manoeuvre.active && IMU.IsCollisionDetected())
	{
		IMU.ClearCollisionDetected();
		char collision_message[100];
		sprintf(collision_message, "Collision in state %s", StateName(eFSM_state));
		logger.Log(collision_message, LOGLEVEL_INFO);
		// Hit while searching means the opponent has contact but our sensors don't see them:
		// spin in place (search 1) to reacquire instead of driving on blindly.
		last_detection_tick = xTaskGetTickCount();
		eFSM_state = LOOK_FOR_OPPONENT_1;
	}
	else if(eFSM_state == FIGHT && IMU.IsCollisionDetected())
	{
		logger.Log("----  Contact with opponent detected while fighting ----", LOGLEVEL_INFO);
		IMU.ClearCollisionDetected();  // We expected a collision
	}

	// Any state change (incl. start/stop) aborts a running manoeuvre.
	bool state_entry = (eFSM_state != stepped_state);
	stepped_state = eFSM_state;
	if(state_entry)
	{
		manoeuvre.End();
		LED2_OFF;
	}
	switch(eFSM_state)
	{
		case CALIBRATE:
		{
			Calibrate();
			eFSM_state = WAIT_FOR_START;
			#ifdef DEBUG_SCREEN
			display.setCursor(0,0);
			display.println("State: WAIT FOR START");
			#endif
			IMU.ResetAngularOrientation();
		}
		break;
		case WAIT_FOR_START:
		{
			last_detection_tick = xTaskGetTickCount();
			MCInterface::SetMotorsPower(0, 0);
			if(!direction_ack)
			{
				if(IMU.GetAngularOrientationForAxis(2) > 60)
				{
					lastDetOnLeft = false;
					direction_ack = true;
				}
				else if (IMU.GetAngularOrientationForAxis(2) < -60) {
					lastDetOnLeft = true;
					direction_ack = true;
				}
			}
			else{
				IMU.ResetAngularOrientation();
			}
			IMU.ClearCollisionDetected();
		}
		break;
		case FIGHT:
		{

			direction_ack = true;
			// Lost only on a fresh sweep: entering from LookForOpponent2 leaves the flag stale.
			if(new_tof_data && !opponent_detected)
			{
				eFSM_state = LOOK_FOR_OPPONENT_1;
				last_detection_tick = xTaskGetTickCount();
			}
			else
			{
				FightAlgorithm();
			}
			//Then check OOB condition
			if(isOutOfBounds())
			{
				eFSM_state = OUT_OF_BOUNDS;
			}
		}
		break;
		case OUT_OF_BOUNDS:
		{
			// !active: safety net, never sit in OOB without a running escape.
			if(state_entry || !manoeuvre.active)
			{
				LED2_ON;
				MCInterface::SetMotorsPower(0, 0);
				manoeuvre.EnterPhase(OOB_STOP, OOB_STOP_MS);
				char oob_message[100];
				sprintf(oob_message, " Out Of Bounds: line %s%s, escape turn %s",
						(LineDetOnLeft())? "L" : "", (LineDetOnRight())? "R" : "",
						(OobTurnLeft())? "LEFT" : "RIGHT");
				logger.Log(oob_message, LOGLEVEL_INFO);
			}
			else if(manoeuvre.PhaseDone())
			{
				switch(manoeuvre.phase)
				{
					case OOB_STOP:
						MCInterface::SetMotorsPower(-NeuroParams.oob_back_speed, -NeuroParams.oob_back_speed);
						manoeuvre.EnterPhase(OOB_BACK, OOB_BACK_MS);
						break;
					case OOB_BACK:
						if(OobTurnLeft())
							MCInterface::SetMotorsPower(-NeuroParams.oob_turn_speed, NeuroParams.oob_turn_speed);
						else
							MCInterface::SetMotorsPower(NeuroParams.oob_turn_speed, -NeuroParams.oob_turn_speed);
						manoeuvre.EnterPhase(OOB_TURN, OOB_TURN_MS);
						break;
					default:   // OOB_TURN done
						manoeuvre.End();
						if(opponent_detected)
						{
							eFSM_state = FIGHT;
						}
						else
						{
							eFSM_state = LOOK_FOR_OPPONENT_1;
							last_detection_tick = xTaskGetTickCount();
						}
						break;
				}
			}
		}
		break;
		case LOOK_FOR_OPPONENT_1:
		{
			double SteeringTrim = (!lastDetOnLeft) ? NeuroParams.search_spin_speed : -NeuroParams.search_spin_speed;
			if(opponent_detected)
			{
				// Counter-turn; FightAlgorithm overrides it on the next (immediate) pass.
				eFSM_state = FIGHT;
				MCInterface::SetMotorsPower(-SteeringTrim, SteeringTrim);
				vTaskDelay(10);
			}
			else
			{
				this->SetFlashPeriodMS(1000);

				
				MCInterface::SetMotorsPower(SteeringTrim, -SteeringTrim);

				// Wait SEARCH_STRATEGY_CHANGE_TIME before changing strategy
				if((xTaskGetTickCount() - last_detection_tick) < SEARCH_STRATEGY_CHANGE_TIME)
				{
					eFSM_state = LOOK_FOR_OPPONENT_1;
				}
				else
				{
					eFSM_state = LOOK_FOR_OPPONENT_2;
				}
			}
			eFSM_state = (isOutOfBounds())? OUT_OF_BOUNDS : eFSM_state;
		}
		break;
		case LOOK_FOR_OPPONENT_2:
		{
			eFSM_state = neuro_v1::LookForOpponent2(this, &manoeuvre, opponent_detected);
		}
		break;
		case NOP:
			break;
		default:
			eFSM_state = WAIT_FOR_START;
			break;
	}
	run_again = (eFSM_state != stepped_state);
	// A state set by the switch is always an entry next pass, even if the pre-switch
	// checks set it straight back to the state handled here (OOB -> LOOK_1 -> OOB).
	if(run_again)
	{
		stepped_state = NOP;
	}

	// Log every FSM transition so match logs show the full decision timeline
	static State_t last_logged_state = CALIBRATE;
	if(last_logged_state != eFSM_state)
	{
		char state_message[100];
		sprintf(state_message, "FSM: %s -> %s", StateName(last_logged_state), StateName(eFSM_state));
		logger.Log(state_message, LOGLEVEL_INFO);
		last_logged_state = eFSM_state;
	}
}

void MiniNeuroV1::end(void)
{
//		Error_Handler();
// Facilitate reset
}

// HAL_GPIO_EXTI_Callback() and HAL_TIM_PWM_PulseFinishedCallback(): RobotHardware.cpp.
void MiniNeuroV1::PeriodicCheckCall(void)
{
	static uint8_t call_count = 0;
	static uint16_t flash_counter = flash_period_ms/2;
	static uint16_t current_fc = flash_period_ms;
	static uint32_t start_tick = xTaskGetTickCount();
	static uint8_t diver = 0;
	static uint8_t reset_counter = 0;

	if(current_fc != flash_period_ms)
	{
		flash_counter = flash_period_ms/2;
		current_fc = flash_period_ms;
	}
	if(!isInitCompleted())
	{
		if((xTaskGetTickCount() - start_tick) > INIT_TIMEOUT_MS)
		{
			if(!this->isRunning() && was_running)
			{
				this->run();
			}
			else{
				if(reset_counter >= 1)
				{
					// Flush(), not Sync(): the staged sub-sector tail has to reach the
					// card before the reset, even though it costs the write alignment.
					logger.Flush();
					NVIC_SystemReset();
				}
				reset_counter++;
				this->forceKill();
				ToF_Sensor::KillSensors();
				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
				start_tick = xTaskGetTickCount();
			}
		}

	}
	// we could use mod, but that might not be fast for architectures other than ARM
	switch(call_count)
	{
		case 25:
			{
//				MCInterface::RunStateCheck();
			}
			break;
		case 50:
			{
//				MCInterface::RunStateCheck();
			}
			break;
		case 75:
			{
//				MCInterface::RunStateCheck();
			}
			break;
		case 100:
			{
//				MCInterface::RunStateCheck();

				if(diver == 5){
					logger.Sync();
//					HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
					
				}
				#ifdef DEBUG_SCREEN
				DisplayThread.RequestUpdate();
				#endif
				diver = (diver >= 5)? 0: diver+1;
			}
			break;
		default:
			break;
	}
	call_count = (call_count >= 100)? 0: call_count+1;
	if(flash_counter == 0){
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
	}
	flash_counter = (flash_counter == 0)? flash_period_ms/2: flash_counter-1;

	// New line detection wakes Robot::loop() without waiting for the next ToF sample.
	static bool line_prev = false;
	bool line_now = LineSensors.Read() != 0;
	EventGroupHandle_t tof_events = ToF_Sensor::GetEventHandle();
	if(line_now && !line_prev && (tof_events != NULL))
	{
		xEventGroupSetBits(tof_events, LINE_EVENT_BIT);
	}
	line_prev = line_now;

// PID calculations
	UpdateHeadingCorrection((float)IMU.GetAngularOrientationForAxis(2));
// Heap check
	
}

void MiniNeuroV1::PeriodCB(void)
{
	MOTOR_CONTROLLERS[MOTOR_LEFT].SoftPWMCB_period();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SoftPWMCB_period();
}
Robot* CreateMiniNeuroV1(void)
{
	return new MiniNeuroV1();
}

#endif
