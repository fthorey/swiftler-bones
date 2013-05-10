#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "hardware.h"
#include "leds.h"

# define LEDS_CNT 1

static led_t leds[LEDS_CNT] = {{.GPIOx = GPIOA, .GPIO_Pin_x = GPIO_Pin_5}}; // LED_GREEN

void led_init()
{
  for (int i = 0; i < LEDS_CNT; i++) {
    gpio_clock_init(leds[i].GPIOx);

    GPIO_InitTypeDef init =
      {
        .GPIO_Pin   = leds[i].GPIO_Pin_x,
        .GPIO_Speed = GPIO_Speed_2MHz,
        .GPIO_Mode  = GPIO_Mode_Out_PP,
      };

    GPIO_Init(leds[i].GPIOx, &init);
    GPIO_ResetBits(leds[i].GPIOx, leds[i].GPIO_Pin_x);
  }
}

void led_off(enum eLED led)
{
  GPIO_ResetBits(leds[led].GPIOx, leds[led].GPIO_Pin_x);
}

void led_on(enum eLED led)
{
  GPIO_SetBits(leds[led].GPIOx, leds[led].GPIO_Pin_x);
}

void led_toggle(enum eLED led)
{
  if (GPIO_ReadOutputDataBit(leds[led].GPIOx, leds[led].GPIO_Pin_x))
    led_off(led);
  else
    led_on(led);
}
