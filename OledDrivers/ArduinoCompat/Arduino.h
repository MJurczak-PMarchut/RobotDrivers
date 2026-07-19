/*
 * Arduino.h
 *
 * Minimal Arduino-core shim so the vendored Adafruit-GFX-Library / Adafruit_SSD1327
 * submodules (RobotDrivers/OledDrivers/{Adafruit-GFX-Library,Adafruit_SSD1327}) compile
 * on bare-metal STM32 HAL + FreeRTOS. Only implements the exact symbols those libraries
 * reference - not a general-purpose Arduino core. See ArduinoCompat/Adafruit_I2CDevice.*
 * and ArduinoCompat/Adafruit_SPIDevice.* for how bus traffic actually reaches hardware
 * (through this project's own CommManager, not through Wire/SPI transactions).
 */

#ifndef ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_ARDUINO_H_
#define ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_ARDUINO_H_

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <type_traits>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Binary.h"

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Real function templates, not object-like macros: Adafruit_GFX.h pulls in <string>
// (via Print.h) right after this header, and object-like `#define min(a,b)`/`max(a,b)`
// macros (as real Arduino cores use) would corrupt libstdc++'s own `max()`/`min()`
// member functions (e.g. bits/max_size_type.h) wherever that identifier appears
// afterwards in the same translation unit - a well-known Arduino/STL collision.
// Templates give the same call-site flexibility (mixed integer types, used
// unqualified as bare min()/max()) without touching the preprocessor.
//
// Return type is explicitly std::common_type_t, not a trailing decltype(a<b?a:b):
// when T1==T2, the ternary of two same-typed lvalues is itself an lvalue, so
// decltype would deduce a *reference* to the by-value parameter - a dangling
// reference to a local the moment the caller uses the result (caught by GCC's
// -Wdangling-pointer during initial testing).
template <typename T1, typename T2>
inline std::common_type_t<T1, T2> min(T1 a, T2 b)
{
	return (a < b) ? a : b;
}

template <typename T1, typename T2>
inline std::common_type_t<T1, T2> max(T1 a, T2 b)
{
	return (a > b) ? a : b;
}

// No AVR/ESP-style flash-vs-RAM address space on STM32, so PROGMEM/pgm_read_* are
// all no-ops here (see also Adafruit_GFX.cpp's own pgm_read_byte/word/dword fallback
// macros, self-defined with `#ifndef` guards for the same reason).
#ifndef PROGMEM
#define PROGMEM
#endif

// Arduino-style "pin number" encoding used throughout ArduinoCompat/: portIndex*16 +
// pinNumber (portIndex 0=GPIOA, 1=GPIOB, ...). ARDUINO_NO_PIN (-1) means "not
// connected" and is treated as a no-op everywhere it's checked (pinMode, digitalWrite,
// Adafruit_SPIDevice's CS handling). Nothing outside this shim needs to know the encoding.
constexpr int16_t ARDUINO_NO_PIN = -1;

inline constexpr int16_t ArduinoPin(uint8_t portIndex, uint8_t pinNumber)
{
	return (int16_t)(portIndex * 16 + pinNumber);
}

// Resolves an encoded pin to its GPIO_TypeDef*/bit mask. Returns false (leaving *port/*mask
// untouched) for ARDUINO_NO_PIN or an out-of-range encoding - callers must check the result.
bool ArduinoPinDecode(int16_t pin, GPIO_TypeDef **port, uint16_t *mask);

// pin is expected to already be configured as GPIO_OUTPUT via CubeMX/HAL_GPIO_Init before
// use (matches this project's existing convention for fixed CS pins, e.g. LSM6DSO_nCS_PIN) -
// this just reconfigures it, mirroring Arduino's pinMode() semantics for callers that expect it.
void pinMode(int16_t pin, uint8_t mode);
void digitalWrite(int16_t pin, uint8_t value);

uint32_t millis(void);
void delay(uint32_t ms);
inline void yield(void) {}
inline double radians(double deg) { return deg * 0.017453292519943295; }

#endif /* ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_ARDUINO_H_ */
