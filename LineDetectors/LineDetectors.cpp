/*
 * LineDetectors.cpp
 *
 *  Created on: 2 lis 2022
 *      Author: Mateusz
 */

#include "LineDetectors.hpp"


LineDetectors::LineDetectors(uint8_t no_of_detectors)
{
	if(no_of_detectors <= MAX_LINE_DETECTOR_COUNT)
	{
		this->__detector_memory = (uint32_t*)malloc(no_of_detectors*sizeof(uint32_t));
		this->__no_of_detectors = no_of_detectors;
	}
	else
	{
		this->__detector_memory = 0;
		Error_Handler();
	}
}

LineDetectors::~LineDetectors()
{
	free(this->__detector_memory);
}

HAL_StatusTypeDef LineDetectors::AttachDetectors(ADC_HandleTypeDef *hadc, LineDetectors_t* DetectorNamesEnums, uint8_t no_of_detectors_to_attach)
{
	if(this->__no_of_detectors < (this->__attached_detectors + no_of_detectors_to_attach)){
		return HAL_ERROR;
	}
	else{
		for(uint8_t u8Iter = 0; u8Iter < no_of_detectors_to_attach; u8Iter++)
		{
			this->__translation_table[DetectorNamesEnums[u8Iter]] = &this->__detector_memory[this->__attached_detectors];
			this->__attached_detectors++;
		}
		return HAL_ADC_Start_DMA(hadc, this->__detector_memory + (this->__attached_detectors - no_of_detectors_to_attach), no_of_detectors_to_attach);
	}
	return HAL_ERROR;
}

uint32_t LineDetectors::GetDetectorValue(LineDetectors_t DetectorEnum)
{
	return *this->__translation_table[DetectorEnum];
}
