#include <stdint.h>
#include "led.h"

void delay(uint32_t count)
{
  for(uint32_t i = 0 ; i < count ; i++);
}

void led_init(void)
{

	uint32_t *pAhb1En = RCC_AHB1ENR;
	uint32_t *pGpioDMode = GPIOD_MODER;

	*pAhb1En |= ( 1 << 3);

	// Configure LEDs
	*pGpioDMode |= ( 1 << (2 * LED_GREEN));
	*pGpioDMode |= ( 1 << (2 * LED_ORANGE));
	*pGpioDMode |= ( 1 << (2 * LED_RED));
	*pGpioDMode |= ( 1 << (2 * LED_BLUE));

    led_off(LED_GREEN);
    led_off(LED_ORANGE);
    led_off(LED_RED);
    led_off(LED_BLUE);
}

void led_on(uint8_t led_no)
{
	uint32_t *pGpioDData = GPIOD_ODR;
	*pGpioDData |= ( 1 << led_no);
}

void led_off(uint8_t led_no)
{
	uint32_t *pGpioDData = GPIOD_ODR;
	*pGpioDData &= ~( 1 << led_no);
}
