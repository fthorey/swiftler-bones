#ifndef SHARPS_H
# define SHARPS_H

#include "FreeRTOS.h"

#define SHARPS_NB 2

#define SHARP_LEFT  0
#define SHARP_RIGHT 1

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
  uint8_t ADC_Channel_x;
} sharp_t;

typedef struct
{
  ADC_TypeDef* ADCx;
  sharp_t ADCs[SHARPS_NB];
  DMA_TypeDef* DMAx;
  DMA_Channel_TypeDef* DMA_Channelx;
} sharps_t;

void vSharpsInit();
float iSharpsMeasureDistCm(int sharp_);

#endif
