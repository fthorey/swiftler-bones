#include "leds.h"
#include "libperiph/leds.h"

static led_switch_t leds = {.GPIOx = GPIOA, .GPIO_Pin_x = GPIO_Pin_5}; // LED_Rn

void vLEDsInit()
{
  for (int i = 0; i < 1; i++)
    led_switch_init(&leds);
}

void vLEDOn(enum eColorLED eLED)
{
  led_switch_on(&leds);
}

void vLEDOff(enum eColorLED eLED)
{
  led_switch_off(&leds);
}

void vLEDToggle(enum eColorLED eLED)
{
  led_switch_toggle(&leds);
}

void vLEDSetColor(enum eColor eLEDColor)
{
  for (int i = 0; i < 1; i++) {
    if (eLEDColor & (1 << i))
        led_switch_on(&leds);
    else
      led_switch_off(&leds);
  }
}


