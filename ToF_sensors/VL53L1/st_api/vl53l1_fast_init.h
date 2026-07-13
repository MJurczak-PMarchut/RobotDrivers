/*
 * vl53l1_fast_init.h
 *
 * Single entry point into the vendored ST VL53L1 full API (st_api/): one-time
 * configuration of a sensor for AN5263 fast ranging at ~100 Hz. Keeps the full
 * API's headers (which conflict with the ULD driver's) out of vl53l1x.cpp.
 */

#ifndef ST_API_VL53L1_FAST_INIT_H_
#define ST_API_VL53L1_FAST_INIT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Configures the (already address-programmed, booted) sensor for AN5263 fast
 * ranging - LITE_RANGING preset, short distance mode, 10 ms timing budget,
 * back-to-back streaming - and starts ranging. The interrupt line is set to
 * active-high "new sample ready", matching what the EXTI wiring and the ULD
 * readout path expect. Blocking; call from task context only.
 * Returns 0 on success, negative VL53L1_Error otherwise. */
int8_t VL53L1_FastRangingInit(uint16_t i2c_addr_8bit);

#ifdef __cplusplus
}
#endif

#endif /* ST_API_VL53L1_FAST_INIT_H_ */