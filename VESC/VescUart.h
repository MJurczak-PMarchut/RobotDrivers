#ifndef _VESCUART_h
#define _VESCUART_h

#include "datatypes.h"
#include "buffer.h"
#include "crc.h"
#include "Configuration.h"

class VescUart
{
	/** Struct to store the telemetry data returned by the VESC */
	struct dataPackage {
		float avgMotorCurrent;
		float avgInputCurrent;
		float dutyCycleNow;
		long rpm;
		float inpVoltage;
		float ampHours;
		float ampHoursCharged;
		long tachometer;
		long tachometerAbs;
	};



	public:
		/**
		 * @brief      Class constructor
		 */
	    VescUart(UART_HandleTypeDef *huart);

		/** Variabel to hold measurements returned from VESC */
		dataPackage data;

		/**
		 * @brief      Set the serial port for debugging
		 * @param      port  - Reference to Serial port (pointer)
		 */

		bool getVescValues(void);

		/**
		 * @brief      Sends values for joystick and buttons to the nunchuck app
		 */
		void setNunchuckValues(void);

		/**
		 * @brief      Set the current to drive the motor
		 * @param      current  - The current to apply
		 */
		void setCurrent(float current);

		/**
		 * @brief      Set the current to drive the motor
		 * @param      current  - The current to apply
		 */
		void setCurrent(float current, int motorId);

		/**
		 * @brief      Set the current to brake the motor
		 * @param      brakeCurrent  - The current to apply
		 */
		void setBrakeCurrent(float brakeCurrent);

		/**
		 * @brief      Set the current to brake the motor
		 * @param      brakeCurrent  - The current to apply
		 */
		void setBrakeCurrent(float brakeCurrent, int motorId);

		/**
		 * @brief      Set the rpm of the motor
		 * @param      rpm  - The desired RPM (actually eRPM = RPM * poles)
		 */
		void setRPM(float rpm);

		/**
		 * @brief      Set the rpm of the motor
		 * @param      rpm  - The desired RPM (actually eRPM = RPM * poles)
		 */
		void setRPM(float rpm, int motorId);

		/**
		 * @brief      Set the duty of the motor
		 * @param      duty  - The desired duty (0.0-1.0)
		 */
		void setDuty(float duty);

		/**
		 * @brief      Set the duty of the motor
		 * @param      duty  - The desired duty (0.0-1.0)
		 */
		void setDuty(float duty, int motorId);

		/**
		 * @brief      Packs the payload and sends it over Serial
		 *
		 * @param      payload  - The payload as a unit8_t Array with length of int lenPayload
		 * @param      lenPay   - Length of payload
		 * @return     The number of bytes send
		 */
		int packSendPayload(uint8_t * payload, int lenPay);

	private:

		UART_HandleTypeDef *__huart = NULL;

		/**
		 * @brief      Receives the message over Serial
		 *
		 * @param      payloadReceived  - The received payload as a unit8_t Array
		 * @return     The number of bytes receeived within the payload
		 */
		int receiveUartMessage(uint8_t * payloadReceived);

		/**
		 * @brief      Verifies the message (CRC-16) and extracts the payload
		 *
		 * @param      message  - The received UART message
		 * @param      lenMes   - The lenght of the message
		 * @param      payload  - The final payload ready to extract data from
		 * @return     True if the process was a success
		 */
		bool unpackPayload(uint8_t * message, int lenMes, uint8_t * payload);

		/**
		 * @brief      Extracts the data from the received payload
		 *
		 * @param      message  - The payload to extract data from
		 * @return     True if the process was a success
		 */
		bool processReadPacket(uint8_t * message);

};

#endif
