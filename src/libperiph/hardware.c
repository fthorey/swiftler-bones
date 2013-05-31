#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "hardware.h"

void vHardwareInit()
{
  // Enable HSE:
  RCC_HSEConfig(RCC_HSE_ON);

  // Wait for HSE to be ready:
  while (RCC_WaitForHSEStartUp() != SUCCESS);

  // Set PLL to be 9 * HSE = 72 MHz:
  RCC_PLLCmd(DISABLE);
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
  RCC_PLLCmd(ENABLE);

  // Wait for PLL to be ready:
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET);

  // Two wait states, if 48 MHz < SYSCLK <= 72 MHz:
  FLASH_SetLatency(FLASH_Latency_2);

  // Set PLL as system clock:
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  // Disable HSI:
  RCC_HSICmd(DISABLE);

  // Set APB low-speed clock (PCLK1), divide by 2:
  RCC_PCLK1Config(RCC_HCLK_Div2);

  // Set APB high-speed clock (PCLK2), do not divide:
  RCC_PCLK2Config(RCC_HCLK_Div1);

  // Set AHB clock (HCLK), do not divide:
  RCC_HCLKConfig(RCC_SYSCLK_Div1);

  // 3 bits for pre-emption priority 1 bits for subpriority:
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

  // Set core clock as SYSTICK source:
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

#ifdef RAM_BOOT
  // Put vector interrupt table in RAM:
  NVIC_SetVectorTable(NVIC_VectTab_RAM, SCB_VTOR_TBLBASE);
#endif
}

#define GPIO_CASE(GPIO)                                          \
  case (uint32_t)GPIO:                                           \
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_##GPIO, ENABLE);         \
  break                                                          \

void vGpioClockInit(GPIO_TypeDef* GPIOx_)
{
  switch((uint32_t)GPIOx_)
  {
    GPIO_CASE(GPIOA);
    GPIO_CASE(GPIOB);
    GPIO_CASE(GPIOC);
    GPIO_CASE(GPIOD);
    GPIO_CASE(GPIOE);
    GPIO_CASE(GPIOF);
    GPIO_CASE(GPIOG);
  }
}

#undef GPIO_CASE

#define TIMER_CASE81(TIM)                                  \
  case (uint32_t)TIM:                                      \
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_##TIM,  ENABLE);   \
  break                                                    \

#define TIMER_CASE27(TIM)                                  \
  case (uint32_t)TIM:                                      \
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_##TIM,  ENABLE);   \
  break                                                    \

void vTimerClockInit(TIM_TypeDef* TIMz_)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
  switch((uint32_t)TIMz_)
  {
    TIMER_CASE81(TIM1);
    TIMER_CASE27(TIM2);
    TIMER_CASE27(TIM3);
    TIMER_CASE27(TIM4);
    TIMER_CASE27(TIM5);
    TIMER_CASE27(TIM6);
    TIMER_CASE27(TIM7);
    TIMER_CASE81(TIM8);
  }
}

#undef TIMER_CASE27
#undef TIMER_CASE81

void vDmaClockInit(DMA_TypeDef* DMAx_)
{
  if (DMAx_ == DMA1)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  else if (DMAx_ == DMA2)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
}

void vAdcClockInit(ADC_TypeDef* ADCx_)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
  if (ADCx_ == ADC1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
  else if (ADCx_ == ADC2)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,  ENABLE);
  else if (ADCx_ == ADC3)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,  ENABLE);
}

void vSpiClockInit(SPI_TypeDef* SPIx_)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  if (SPIx_ == SPI1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  else if (SPIx_ == SPI2)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
}

void vCanClockInit(CAN_TypeDef* CANx_)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
}

void vI2CClockInit(I2C_TypeDef* I2Cx_)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  if (I2Cx_ == I2C1)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  else if (I2Cx_ == I2C2)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
}

void vWaitUs(int x_)
{
  int i, j;
  for(i = 0; i < x_; i++)
    for(j = 0; j < 50; j++);
}
