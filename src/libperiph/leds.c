#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "hardware.h"
#include "leds.h"

# define LEDS_MAX_CNT 0xffff

void led_switch_init(led_switch_t* led)
{
  gpio_clock_init(led->GPIOx);

  GPIO_InitTypeDef init =
    {
      .GPIO_Pin   = led->GPIO_Pin_x,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_Out_PP,
    };

  GPIO_Init(led->GPIOx, &init);
  led_switch_off(led);
}

void led_pwm_init(led_pwm_t* led)
{
  gpio_clock_init(led->GPIOx);

  GPIO_InitTypeDef init =
    {
      .GPIO_Pin   = led->GPIO_Pin_x,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_AF_PP,
    };

  GPIO_Init(led->GPIOx, &init);

  timer_clock_init(led->TIMx);

  TIM_Cmd(led->TIMx, DISABLE);

  TIM_TimeBaseInitTypeDef Timer_InitStructure;
  Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  Timer_InitStructure.TIM_Period = LEDS_MAX_CNT;
  Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

  TIM_TimeBaseInit(led->TIMx, &Timer_InitStructure);

  TIM_OCInitTypeDef OC_InitStructure;
  TIM_OCStructInit(&OC_InitStructure);
  OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM2;

  switch (led->TIM_Channel)
  {
    case TIM_Channel_1:
      TIM_OC1Init(led->TIMx, &OC_InitStructure);
      TIM_OC1PreloadConfig(led->TIMx, TIM_OCPreload_Enable);
      break;
    case TIM_Channel_2:
      TIM_OC2Init(led->TIMx, &OC_InitStructure);
      TIM_OC2PreloadConfig(led->TIMx, TIM_OCPreload_Enable);
      break;
    case TIM_Channel_3:
      TIM_OC3Init(led->TIMx, &OC_InitStructure);
      TIM_OC3PreloadConfig(led->TIMx, TIM_OCPreload_Enable);
      break;
    case TIM_Channel_4:
      TIM_OC4Init(led->TIMx, &OC_InitStructure);
      TIM_OC4PreloadConfig(led->TIMx, TIM_OCPreload_Enable);
      break;
  }

  TIM_CCxCmd(led->TIMx, led->TIM_Channel, TIM_CCx_Enable);

  if (led->TIMx == TIM1 || led->TIMx == TIM8)
    TIM_CtrlPWMOutputs(led->TIMx, ENABLE);

  TIM_ARRPreloadConfig(led->TIMx, ENABLE);
  TIM_Cmd(led->TIMx, ENABLE);

  led_pwm_intensity(led, 0);
}

void led_pwm_intensity(led_pwm_t* led, uint8_t value)
{
  const uint16_t compare = value * LEDS_MAX_CNT / 0xff;

  switch (led->TIM_Channel)
  {
    case TIM_Channel_1:
      TIM_SetCompare1(led->TIMx, compare);
      break;
    case TIM_Channel_2:
      TIM_SetCompare2(led->TIMx, compare);
      break;
    case TIM_Channel_3:
      TIM_SetCompare3(led->TIMx, compare);
      break;
    case TIM_Channel_4:
      TIM_SetCompare4(led->TIMx, compare);
      break;
  }
}

void led_switch_off(led_switch_t* led)
{
  GPIO_ResetBits(led->GPIOx, led->GPIO_Pin_x);
}

void led_switch_on(led_switch_t* led)
{
  GPIO_SetBits(led->GPIOx, led->GPIO_Pin_x);
}

void led_switch_toggle(led_switch_t* led)
{
  if (GPIO_ReadOutputDataBit(led->GPIOx, led->GPIO_Pin_x))
    led_switch_off(led);
  else
    led_switch_on(led);
}
