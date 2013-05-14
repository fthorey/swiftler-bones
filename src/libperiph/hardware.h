#ifndef LIBPERIPH_HARDWARE_H
# define LIBPERIPH_HARDWARE_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"

#define MS_TO_TICKS(time_ms) ((portTickType)((time_ms) / portTICK_RATE_MS))

#define GPIO_TO_EXTI_LINE(GPIO_Pin) (GPIO_Pin)

void vHardwareInit();
void vGpioClockInit(GPIO_TypeDef* GPIOx_);
void vTimerClockInit(TIM_TypeDef* TIMx_);
void vDmaClockInit(DMA_TypeDef* DMAx_);
void vAdcClockInit(ADC_TypeDef* ADCx_);
void vSpiClockInit(SPI_TypeDef* SPIx_);
void vCanClockInit(CAN_TypeDef* CANx_);
void vWaitUs(int x_);

#endif
