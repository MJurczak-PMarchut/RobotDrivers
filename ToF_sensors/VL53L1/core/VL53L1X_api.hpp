/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file : part of VL53L1 Core and : dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document : strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/

/**
 * @file  vl53l1x_api.h
 * @brief Functions definition
 */

#ifndef _API_H_
#define _API_H_

#include "vl53l1_platform.hpp"
#include "ToFSensor.hpp"

#define VL53L1X_IMPLEMENTATION_VER_MAJOR       3
#define VL53L1X_IMPLEMENTATION_VER_MINOR       5
#define VL53L1X_IMPLEMENTATION_VER_SUB         1
#define VL53L1X_IMPLEMENTATION_VER_REVISION  0000

typedef uint8_t VL53L1X_ERROR;

#define SOFT_RESET											0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS					0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 		0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 	0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 	0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM					0x001E
#define MM_CONFIG__INNER_OFFSET_MM							0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 							0x0022
#define GPIO_HV_MUX__CTRL									0x0030
#define GPIO__TIO_HV_STATUS       							0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO 						0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP     				0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI   				0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A        				0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B						0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI  					0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO  					0x0062
#define RANGE_CONFIG__SIGMA_THRESH 							0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS			0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH      				0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD				0x006C
#define SYSTEM__THRESH_HIGH 								0x0072
#define SYSTEM__THRESH_LOW 									0x0074
#define SD_CONFIG__WOI_SD0                  				0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0        				0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD					0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE		0x0080
#define SYSTEM__SEQUENCE_CONFIG								0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD 				0x0082
#define SYSTEM__INTERRUPT_CLEAR       						0x0086
#define SYSTEM__MODE_START                 					0x0087
#define VL53L1_RESULT__RANGE_STATUS							0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0		0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD					0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0				0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 	0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL					0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS                      0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                     0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD				0x013E

/****************************************
 * PRIVATE define do not edit
 ****************************************/

//uint8_t VL53L1X_addr [] = {0x5E, 0x5C, 0x5A, 0x58, 0x56, 0x54};

/**
 *  @brief defines SW Version
 */
typedef struct {
	uint8_t      major;    /*!< major number */
	uint8_t      minor;    /*!< minor number */
	uint8_t      build;    /*!< build number */
	uint32_t     revision; /*!< revision number */
} VL53L1X_Version_t;

/**
 *  @brief defines packed reading results type
 */
typedef struct {
	uint8_t Status;		/*!< ResultStatus */
	uint16_t Distance;	/*!< ResultDistance */
	uint16_t Ambient;	/*!< ResultAmbient */
	uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
	uint16_t NumSPADs;	/*!< ResultNumSPADs */
} VL53L1X_Result_t;

class VL53L1X: public ToF_Sensor
{
private:
	static const uint16_t __default_addr = {0x52};
	GPIO_TypeDef *__GPIOx;
	uint16_t __GPIO_Pin;
	MessageInfoTypeDef<I2C_HandleTypeDef> __MsgInfo;
	VL53L1X_ERROR VL53L1X_ClearInterrupt();
	VL53L1X_ERROR VL53L1X_SetI2CAddress();
public:
	VL53L1X(CommManager *comm, e_ToF_Position position, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
	VL53L1X_ERROR VL53L1X_SensorInit();
	VL53L1X_ERROR VL53L1X_SetInterruptPolarity(uint8_t IntPolef);
	virtual ~VL53L1X();
};

/**
 * @brief This function returns the current interrupt polarity\n
 * 1=active high (default), 0=active low
 */
VL53L1X_ERROR VL53L1X_GetInterruptPolarity(uint16_t dev, uint8_t *pIntPol, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);

/**
 * @brief This function starts the ranging distance operation\n
 * The ranging operation is continuous. The clear interrupt has to be done after each get data to allow the interrupt to raise when the next data is ready\n
 * 1=active high (default), 0=active low, use SetInterruptPolarity() to change the interrupt polarity if required.
 */
VL53L1X_ERROR VL53L1X_StartRanging(uint16_t dev, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);

/**
 * @brief This function stops the ranging.
 */
VL53L1X_ERROR VL53L1X_StopRanging(uint16_t dev, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);

/**
 * @brief This function checks if the new ranging data is available by polling the dedicated register.
 * @param : isDataReady==0 -> not ready; isDataReady==1 -> ready
 */
VL53L1X_ERROR VL53L1X_CheckForDataReady(uint16_t dev, uint8_t *isDataReady, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);

/**
 * @brief This function programs the timing budget in ms.
 * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
 */
VL53L1X_ERROR VL53L1X_SetTimingBudgetInMs(uint16_t dev, uint16_t TimingBudgetInMs, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);

/**
 * @brief This function programs the distance mode (1=short, 2=long(default)).
 * Short mode max distance is limited to 1.3 m but better ambient immunity.\n
 * Long mode can range up to 4 m in the dark with 200 ms timing budget.
 */
VL53L1X_ERROR VL53L1X_SetDistanceMode(uint16_t dev, uint16_t DistanceMode, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);

/**
 * @brief This function returns the distance measured by the sensor in mm
 */
VL53L1X_ERROR VL53L1X_GetDistance(uint16_t dev, uint16_t *distance, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);

/**
 * @brief This function returns measurements and the range status in a single read access
 */
VL53L1X_ERROR VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t *pResult, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);


#endif
