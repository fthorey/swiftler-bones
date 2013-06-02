#ifndef SONAR_H
# define SONAR_H

#include <stdint.h>

#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

#include "FreeRTOS.h"

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
  TIM_TypeDef* TIMx;
} sonar_t;

void vSonarInit();
int iSonarMeasureDistCm();
// Should not be called more than 20 times per second.

#endif
