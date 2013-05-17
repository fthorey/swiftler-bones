#ifndef SONAR_H
# define SONAR_H

#include <stdint.h>

#include "FreeRTOS.h"

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
  uint32_t EXTI_Line;
} sonar_t;

void vSonarInit();
int iSonarMeasureDistCm();
// Should not be called more than 20 times per second.

#endif
