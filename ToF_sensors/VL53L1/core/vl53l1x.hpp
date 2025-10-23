/*
 * vl53l1x.h
 *
 *  Created on: Nov 16, 2021
 *      Author: paulina
 */

#ifndef TOF_SENSORS_CORE_VL53L1X_H_
#define TOF_SENSORS_CORE_VL53L1X_H_

#include <vl53l1_api.hpp>
#include "../../Configuration.h"
#include "CommManager.hpp"
#include "ToFSensor.hpp"
#include "vl53l1_platform_user_data.hpp"

#define TOF_DEFAULT_ADDRESS 0x52
#define TIMING_BUDGET_MS         100U
#define MAX_TOF_NUMBER           6U

#define TOF0_Addr                0x54
#define TOF1_Addr                0x56
#define TOF2_Addr                0x58
#define TOF3_Addr                0x5A
#define TOF4_Addr                0x5C
#define TOF5_Addr                0x5E

#define TOF0				     0U
//#define TOF1				     1U
//#define TOF2				     2U
//#define TOF3				     3U
//#define TOF4				     4U
#define TOF5				     5U


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

typedef struct {
	uint8_t Status;		/*!< ResultStatus */
	uint16_t Distance;	/*!< ResultDistance */
	uint16_t Ambient;	/*!< ResultAmbient */
	uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
	uint16_t NumSPADs;	/*!< ResultNumSPADs */
} VL53L1X_Result_t;

//typedef struct
//{
//	bool Active;
//	uint8_t Address;
//	uint16_t GPIO_PIN;
//	GPIO_TypeDef *GPIO_GPIOx;
//	uint16_t XSHUT_PIN;
//	GPIO_TypeDef *XSHUT_GPIOx;
//}VL53L1X_Device;

class VL53L1X: public ToF_Sensor{
public:
	VL53L1X(e_ToF_Position position, CommManager *comm, I2C_HandleTypeDef *hi2c1);
	HAL_StatusTypeDef SetI2CAddress();
	ToF_Status_t CheckSensorStatus(void);
	HAL_StatusTypeDef GetRangingData(void);
	HAL_StatusTypeDef  StartRanging();
	uint16_t GetDistance(void);
	VL53L1X_Result_t GetResult(void);
	uint16_t GetSensorITPin(void);
	HAL_StatusTypeDef __GetData(void);
	HAL_StatusTypeDef DisableSensorComm(void);
	void DataReceived(MessageInfoTypeDef<I2C_HandleTypeDef>* MsgInfo);
	uint8_t ClearInterrupt();
	uint8_t CheckForDataReady(uint8_t *isDataReady);
	uint8_t  GetInterruptPolarity(uint16_t dev, uint8_t *pInterruptPolarity);
	~VL53L1X(void){};
protected:
	HAL_StatusTypeDef SensorInit(void);
	void SetMutex(osapi::Mutex *pmutex);
	HAL_StatusTypeDef SetDistanceMode(uint16_t DM);
	HAL_StatusTypeDef SetTimingBudgetInMs(uint16_t TimingBudgetInMs);
	HAL_StatusTypeDef GetTimingBudgetInMs(uint16_t *pTimingBudget);
	HAL_StatusTypeDef GetDistanceMode(uint16_t *DM);
	HAL_StatusTypeDef SetInterMeasurementInMs(uint32_t InterMeasMs);
private:
	uint8_t __comm_buffer[20];
	I2C_HandleTypeDef *__hi2c1;
	static uint8_t __sensor_nb;
	VL53L1X_Result_t Result;
	uint8_t __sensor_index;
	TickType_t __wait_until_tick;
	ToF_Status_t __Status;
	uint16_t __distance;
	uint32_t __data_count;
	uint16_t __address = TOF_DEFAULT_ADDRESS;
	std::function<void(MessageInfoTypeDef<I2C> *MsgInfo)> _CallbackFunc;
	uint32_t __timing_budget;
	VL53L1_Dev_t dev_struct;
	VL53L1_DEV Dev;
};



#endif /* TOF_SENSORS_CORE_VL53L1X_H_ */
