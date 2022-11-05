/*
 * LineDetectors.hpp
 *
 *  Created on: 2 lis 2022
 *      Author: Mateusz
 */

#ifndef LINEDETECTORS_LINEDETECTORS_HPP_
#define LINEDETECTORS_LINEDETECTORS_HPP_

#include "Configuration.h"
#include <map>

typedef enum {
	LD_FRONT_LEFT = 0,
	LD_FRONT_RIGHT,
	LD_BACK_LEFT,
	LD_BACK_RIGHT
} LineDetectors_t;

/*
 * For now this only supports DMA transfers
 * It is a placeholder for now, will become more advanced when it will be time to implement it for LF
 */

class LineDetectors
{
public:
	LineDetectors(uint8_t no_of_detectors);
	~LineDetectors();
	HAL_StatusTypeDef AttachDetectors(ADC_HandleTypeDef *hadc, LineDetectors_t* DetectorNamesEnums, uint8_t no_of_detectors_to_attach);
	uint32_t GetDetectorValue(LineDetectors_t DetectorEnum);
private:
	uint32_t* __detector_memory;
	uint8_t __attached_detectors;
	uint32_t __zero_value;
	uint8_t __no_of_detectors;
	std::map<LineDetectors_t, uint32_t*> __translation_table; //We could use static allocation with simple translation table, but it should not matter with the H7
};


#endif /* LINEDETECTORS_LINEDETECTORS_HPP_ */
