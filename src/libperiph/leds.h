#ifndef LIBPERIPH_LEDS_H
# define LIBPERIPH_LEDS_H

# include "stm32f10x_gpio.h"
# include "stm32f10x_tim.h"

enum eLED {
  LED_GREEN  = 0,
  LED_YELLOW = 1
};

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
} led_t;

void led_init();
void led_on(enum eLED led);
void led_off(enum eLED led);
void led_toggle(enum eLED led);

#endif
