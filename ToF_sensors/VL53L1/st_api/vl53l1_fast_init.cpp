/*
 * vl53l1_fast_init.cpp
 *
 * AN5263 fast-ranging configuration via the vendored ST full API. This is the
 * only translation unit (besides the platform adapter) that includes the full
 * API headers. The API is used for configuration only: once ranging is started
 * in back-to-back mode the device streams measurements on its own, and the
 * existing lightweight interrupt-driven readout in ../core/vl53l1x.cpp takes
 * over (read result registers + clear interrupt, no per-cycle restart needed
 * as the configuration never changes mid-run).
 */

#include <string.h>
#include "vl53l1_api.h"
#include "vl53l1_fast_init.h"

/* One reusable device-state struct (~few KB): sensors are configured strictly
 * sequentially from the init worker task, and the runtime never touches the
 * full API afterwards, so per-sensor copies would be dead weight. */
static VL53L1_Dev_t FastInitDev;

/* Ranging cadence.
 *
 * Findings from bringing this up on hardware:
 *  - Back-to-back mode (0x20) performs ONE ranging per start command (AN5263: "the
 *    start command enables the next ranging") - sustaining it needs ST's config-block
 *    handshake after every frame, too much I2C traffic for three sensors on one bus.
 *  - Driving TIMED mode with the LITE (streaming-scheduler) preset produces wrong,
 *    unstable periods - the timed presets configure a different scheduler mode
 *    (PSEUDO_SOLO | SINGLE_SD), seed config and inter-measurement handling.
 *  - The API's official SetMeasurementTimingBudget for the AUTONOMOUS (timed) preset
 *    has a fixed 21.6 ms guard, yet the proven ULD register recipe (which IS this
 *    device preset) runs 20 ms periods - the guard is conservative, not physical.
 *
 * Therefore: use the native AUTONOMOUS/TIMED preset (correct scheduler, GPH off,
 * ULD-equivalent behavior) and program the per-phase range timeout directly through
 * ST's low-level encoder (set_timeouts_us), bypassing only the top-level guard.
 *
 * Timing model, verified against device register readback (VL53L1_FastRangingDbg)
 * and measured frame periods:
 *   period = frame + idle
 *   frame ~= phasecal(~1ms) + 4 x RANGE_TIMEOUT (DSS1, DSS2, RANGE A, RANGE B)
 *            + ~1.5ms overhead     [MM stages disabled, matching the ULD blob]
 *   idle  = FAST_RANGING_IDLE_MS (the intermeasurement timer runs on the ~36kHz
 *           low-power oscillator AFTER the frame; it is not the total period)
 * With 2000us/2ms below: period ~= 1 + 8 + 1.5 + 2 ~= 12.5 ms (~80 Hz). */
#define FAST_RANGING_RANGE_TIMEOUT_US    2000
#define FAST_RANGING_IDLE_MS             2

/* Sequence steps (bit0 VHV, bit1 PHASECAL, bit3 DSS1, bit4 DSS2, bit5 MM1,
 * bit6 MM2, bit7 RANGE):
 *   0xDB - preset default: adds MM2 (~2 ms, frame time varies with target presence)
 *   0x9B - the classic ULD configuration: VHV | PHASECAL | DSS1 | DSS2 | RANGE
 *   0x8B - ST's low-power-auto recipe: keeps DSS1, drops DSS2/MM, DSS in manual
 *          requested-effective-SPADs mode. Bench result: WRONG DISTANCES, then a
 *          freeze - in ST's code this mode is paired with host-side runtime duties
 *          (per-frame DSS updates from results, manual calibration setup after the
 *          first range) that our lightweight readout does not perform. Do not use
 *          without porting that host machinery.
 *   0x83 - DSS stages removed entirely. Bench result: status 2 (signal fail),
 *          garbage distance - the device has no valid SPAD selection.
 * => 0x9B is the fastest configuration verified stable and accurate on this robot:
 *    ~20 ms period (50 Hz) with the 2000 us / 2 ms settings above. */
#define FAST_RANGING_SEQUENCE_CONFIG     0x9B

/* Manual DSS request (8.8 format), only meaningful for the 0x8B experiment. */
#define FAST_RANGING_MANUAL_EFF_SPADS    (200 << 8)

/* Post-init readback of the actual device registers (last-initialized sensor),
 * for Live Watch inspection - decodes whether the fast timeouts really landed. */
typedef struct {
	uint32_t preset_phasecal_us;   /* timeouts reported by the API before our override */
	uint32_t preset_mm_us;
	uint32_t preset_range_us;
	uint16_t nvm_fast_osc;         /* osc_measured__fast_osc__frequency (0x0006) */
	uint16_t reg_osc_calibrate;    /* RESULT__OSC_CALIBRATE_VAL (0x00DE) */
	uint8_t  reg_phasecal_timeout; /* PHASECAL_CONFIG__TIMEOUT_MACROP (0x004B) */
	uint16_t reg_mm_timeout_a;     /* MM_CONFIG__TIMEOUT_MACROP_A_HI (0x005A) */
	uint16_t reg_mm_timeout_b;     /* MM_CONFIG__TIMEOUT_MACROP_B_HI (0x005C) */
	uint16_t reg_range_timeout_a;  /* RANGE_CONFIG__TIMEOUT_MACROP_A_HI (0x005E) */
	uint16_t reg_range_timeout_b;  /* RANGE_CONFIG__TIMEOUT_MACROP_B_HI (0x0061) */
	uint32_t reg_intermeasurement; /* SYSTEM__INTERMEASUREMENT_PERIOD (0x006C) */
	uint8_t  reg_sequence_config;  /* SYSTEM__SEQUENCE_CONFIG (0x0081) */
} VL53L1_FastRangingDbg_t;

volatile VL53L1_FastRangingDbg_t VL53L1_FastRangingDbg;

int8_t VL53L1_FastRangingInit(uint16_t i2c_addr_8bit)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	VL53L1_DEV Dev = &FastInitDev;

	memset(Dev, 0, sizeof(FastInitDev));
	FastInitDev.I2cDevAddr = (uint8_t)i2c_addr_8bit;

	status = VL53L1_WaitDeviceBooted(Dev);
	if (status == VL53L1_ERROR_NONE)
	{
		status = VL53L1_DataInit(Dev);
	}
	if (status == VL53L1_ERROR_NONE)
	{
		status = VL53L1_StaticInit(Dev);
	}
	/* Order matters: preset mode, then distance mode, then timeouts */
	if (status == VL53L1_ERROR_NONE)
	{
		status = VL53L1_SetPresetMode(Dev, VL53L1_PRESETMODE_AUTONOMOUS);
	}
	if (status == VL53L1_ERROR_NONE)
	{
		status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT);
	}
	if (status == VL53L1_ERROR_NONE)
	{
		/* Program the fast range timeout through ST's low-level encoder, keeping the
		 * preset's phasecal/MM timeouts (see rationale at the top of this file). */
		uint32_t phasecal_us = 0, mm_us = 0, range_us = 0;
		status = VL53L1_get_timeouts_us(Dev, &phasecal_us, &mm_us, &range_us);
		VL53L1_FastRangingDbg.preset_phasecal_us = phasecal_us;
		VL53L1_FastRangingDbg.preset_mm_us = mm_us;
		VL53L1_FastRangingDbg.preset_range_us = range_us;
		if (status == VL53L1_ERROR_NONE)
		{
			status = VL53L1_set_timeouts_us(Dev, phasecal_us, mm_us,
					FAST_RANGING_RANGE_TIMEOUT_US);
		}
	}
	if (status == VL53L1_ERROR_NONE)
	{
		status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, FAST_RANGING_IDLE_MS);
	}
	if (status == VL53L1_ERROR_NONE)
	{
		/* The full API defaults the interrupt pin to active LOW; the board's EXTI
		 * wiring and the ULD-style readout expect the ULD default (active HIGH,
		 * mux = range/error interrupts, i.e. GPIO_HV_MUX__CTRL = 0x01). Patch the
		 * API's internal static config so StartMeasurement writes the right value. */
		VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
		pdev->stat_cfg.gpio_hv_mux__ctrl =
				VL53L1_DEVICEINTERRUPTPOLARITY_ACTIVE_HIGH |
				VL53L1_DEVICEGPIOMODE_OUTPUT_RANGE_AND_ERROR_INTERRUPTS;

		/* Defensive: the timed preset already disables grouped-parameter-hold, matching
		 * the working ULD register dump; keep it pinned off so a preset change can't
		 * silently reintroduce the per-frame host handshake requirement. */
		pdev->dyn_cfg.system__grouped_parameter_hold_0 = 0x00;
		pdev->dyn_cfg.system__grouped_parameter_hold_1 = 0x00;
		pdev->dyn_cfg.system__grouped_parameter_hold   = 0x00;

		/* Trim the ranging sequence (see FAST_RANGING_SEQUENCE_CONFIG comment). */
		pdev->dyn_cfg.system__sequence_config = FAST_RANGING_SEQUENCE_CONFIG;

#if FAST_RANGING_SEQUENCE_CONFIG == 0x8B
		/* The 0x8B experiment additionally needs DSS in manual mode - and, per the
		 * bench results above, host-side runtime management this driver lacks. */
		pdev->gen_cfg.dss_config__manual_effective_spads_select = FAST_RANGING_MANUAL_EFF_SPADS;
		pdev->gen_cfg.dss_config__roi_mode_control =
				VL53L1_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;
#endif
	}
	if (status == VL53L1_ERROR_NONE)
	{
		/* Start via the low-level entry the API itself uses: VL53L1_StartMeasurement
		 * only adds the conservative (budget + 4 ms) guard we deliberately bypass. */
		status = VL53L1_init_and_start_range(Dev,
				VL53L1_DEVICEMEASUREMENTMODE_TIMED,
				VL53L1_DEVICECONFIGLEVEL_FULL);
	}
	if (status == VL53L1_ERROR_NONE)
	{
		/* Read back what actually landed in the device for Live Watch inspection */
		uint8_t byte_val = 0;
		uint16_t word_val = 0;
		uint32_t dword_val = 0;
		VL53L1_RdWord(Dev, 0x0006, &word_val);
		VL53L1_FastRangingDbg.nvm_fast_osc = word_val;
		VL53L1_RdWord(Dev, 0x00DE, &word_val);
		VL53L1_FastRangingDbg.reg_osc_calibrate = word_val;
		VL53L1_RdByte(Dev, 0x004B, &byte_val);
		VL53L1_FastRangingDbg.reg_phasecal_timeout = byte_val;
		VL53L1_RdWord(Dev, 0x005A, &word_val);
		VL53L1_FastRangingDbg.reg_mm_timeout_a = word_val;
		VL53L1_RdWord(Dev, 0x005C, &word_val);
		VL53L1_FastRangingDbg.reg_mm_timeout_b = word_val;
		VL53L1_RdWord(Dev, 0x005E, &word_val);
		VL53L1_FastRangingDbg.reg_range_timeout_a = word_val;
		VL53L1_RdWord(Dev, 0x0061, &word_val);
		VL53L1_FastRangingDbg.reg_range_timeout_b = word_val;
		VL53L1_RdDWord(Dev, 0x006C, &dword_val);
		VL53L1_FastRangingDbg.reg_intermeasurement = dword_val;
		VL53L1_RdByte(Dev, 0x0081, &byte_val);
		VL53L1_FastRangingDbg.reg_sequence_config = byte_val;
	}
	return (int8_t)status;
}