/*
 * Arduino.cpp
 */
#include "Arduino.h"

namespace
{
GPIO_TypeDef *const kPorts[] = {
	GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH,
#ifdef GPIOI
	GPIOI,
#endif
#ifdef GPIOJ
	GPIOJ,
#endif
#ifdef GPIOK
	GPIOK,
#endif
};
constexpr uint8_t kPortCount = sizeof(kPorts) / sizeof(kPorts[0]);
} // namespace

bool ArduinoPinDecode(int16_t pin, GPIO_TypeDef **port, uint16_t *mask)
{
	if (pin < 0)
	{
		return false;
	}
	uint8_t portIndex = (uint8_t)pin / 16;
	uint8_t pinNumber = (uint8_t)pin % 16;
	if (portIndex >= kPortCount)
	{
		return false;
	}
	*port = kPorts[portIndex];
	*mask = (uint16_t)(1u << pinNumber);
	return true;
}

void pinMode(int16_t pin, uint8_t mode)
{
	GPIO_TypeDef *port;
	uint16_t mask;
	if (!ArduinoPinDecode(pin, &port, &mask))
	{
		return;
	}
	GPIO_InitTypeDef init = {0};
	init.Pin = mask;
	init.Pull = (mode == INPUT_PULLUP) ? GPIO_PULLUP : GPIO_NOPULL;
	init.Speed = GPIO_SPEED_FREQ_LOW;
	init.Mode = (mode == OUTPUT) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
	HAL_GPIO_Init(port, &init);
}

void digitalWrite(int16_t pin, uint8_t value)
{
	GPIO_TypeDef *port;
	uint16_t mask;
	if (!ArduinoPinDecode(pin, &port, &mask))
	{
		return;
	}
	HAL_GPIO_WritePin(port, mask, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint32_t millis(void)
{
	return HAL_GetTick();
}

void delay(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms));
}
