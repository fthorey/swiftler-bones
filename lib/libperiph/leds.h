#ifndef LIBPERIPH_LEDS_H
# define LIBPERIPH_LEDS_H

# include "stm32f10x_gpio.h"
# include "stm32f10x_tim.h"

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
} led_switch_t;

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
  TIM_TypeDef* TIMx;
  uint16_t TIM_Channel;
} led_pwm_t;

void led_pwm_init(led_pwm_t* led);
void led_pwm_intensity(led_pwm_t* led, uint8_t value);

void led_switch_init(led_switch_t* led);
void led_switch_on(led_switch_t* led);
void led_switch_off(led_switch_t* led);
void led_switch_toggle(led_switch_t* led);

#endif
