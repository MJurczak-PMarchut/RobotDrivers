/*
 * Wire.cpp
 */
#include "Wire.h"
#include "CommManager.hpp"

extern "C"
{
	extern I2C_HandleTypeDef hi2c1;
}
// Declared/defined with C linkage in Core/Src/DebugEntryCPP.cpp (inside an
// `extern "C" { }` block) - must be redeclared the same way here to match.
extern "C" CommManager MainCommManager;

// Only stores the addresses of MainCommManager/hi2c1 - both are global objects
// whose addresses are valid regardless of C++ static-init order, and no I2C
// traffic happens until something later calls display.begin(), well after both
// are fully constructed.
TwoWire Wire(&MainCommManager, &hi2c1);
