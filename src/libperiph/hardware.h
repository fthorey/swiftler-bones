#ifndef LIBPERIPH_HARDWARE_H
# define LIBPERIPH_HARDWARE_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"

#define MS_TO_TICKS(time_ms) ((portTickType)((time_ms) / portTICK_RATE_MS))

#define GPIO_TO_EXTI_LINE(GPIO_Pin) (GPIO_Pin)

void hardware_init();
void gpio_clock_init(GPIO_TypeDef* GPIOx);
void timer_clock_init(TIM_TypeDef* TIMx);
void dma_clock_init(DMA_TypeDef* DMAx);
void adc_clock_init(ADC_TypeDef* ADCx);
void spi_clock_init(SPI_TypeDef* SPIx);
void can_clock_init(CAN_TypeDef* CANx);
uint8_t gpio_pin_to_source(uint16_t GPIO_Pin_x);
void wait_us(int x);

#endif
