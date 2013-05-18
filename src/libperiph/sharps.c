#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "misc.h"
#include "semphr.h"

#include "libperiph/hardware.h"
#include "libperiph/sharps.h"

#define AVERAGE_NUMBER 5

static xSemaphoreHandle xSharpsSemaphore;
static volatile uint16_t ADC_DMA_Buffer[SHARPS_MAX_NUMBER];
static volatile uint16_t ADC_Smoothed[SHARPS_MAX_NUMBER];

static sharps_t sharps = { .nb   = 2,
                           .ADCx = ADC1,
                           .ADCs = {{.GPIOx = GPIOC, .GPIO_Pin_x = GPIO_Pin_3, .ADC_Channel_x = ADC_Channel_13},
                                    {.GPIOx = GPIOC, .GPIO_Pin_x = GPIO_Pin_4, .ADC_Channel_x = ADC_Channel_14}},
                           .DMAx = DMA1,
                           .DMA_Channelx = DMA1_Channel1 };

static void vSharpsSmoothTask(void* pvParameters_);

void vSharpsInit(unsigned portBASE_TYPE sharpsDaemonPriority_)
{
  // Configure DMA clock
  vDmaClockInit(sharps.DMAx);
  // Configure ADC clock
  vAdcClockInit(sharps.ADCx);

  // Default GPIO config
  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin = 0,
      .GPIO_Speed = GPIO_Mode_IN_FLOATING,
      .GPIO_Mode = GPIO_Speed_2MHz,
    };

  // GPIO config
  for (int i = 0; i < sharps.nb; i++)
  {
    vGpioClockInit(sharps.ADCs[i].GPIOx);
    GPIO_InitStructure.GPIO_Pin = sharps.ADCs[i].GPIO_Pin_x;
    GPIO_Init(sharps.ADCs[i].GPIOx, &GPIO_InitStructure);
  }

  // Initialize ADC
  ADC_DeInit(sharps.ADCx);
  // Power on ADC
  ADC_Cmd(sharps.ADCx, ENABLE);
  // Wait until it stabilizes
  vWaitUs(1000);

  // Configure ADC
  ADC_InitTypeDef ADC_InitStructure;
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_NbrOfChannel = sharps.nb;
  ADC_Init(sharps.ADCx, &ADC_InitStructure);

  // Calibrate ADC
  ADC_ResetCalibration(sharps.ADCx);
  while (ADC_GetResetCalibrationStatus(sharps.ADCx));
  ADC_StartCalibration(sharps.ADCx);
  while (ADC_GetCalibrationStatus(sharps.ADCx));

  for (int i = 0; i < sharps.nb; i++)
    ADC_RegularChannelConfig(sharps.ADCx, sharps.ADCs[i].ADC_Channel_x,
                             i + 1, ADC_SampleTime_239Cycles5);

  // Start ADC
  ADC_Cmd(sharps.ADCx, ENABLE);

  // Initialize DMA channel
  DMA_DeInit(sharps.DMA_Channelx);
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)sharps.ADCx->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_DMA_Buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = sharps.nb;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 16 ?
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(sharps.DMA_Channelx, &DMA_InitStructure);

  DMA_ITConfig(sharps.DMA_Channelx, DMA_IT_TC, ENABLE);

  // Register DMA interrupt
  NVIC_InitTypeDef NVIC_InitStructure =
    {
      .NVIC_IRQChannel = DMA1_Channel1_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 3, // ?
      .NVIC_IRQChannelSubPriority = 3,        // ?
      .NVIC_IRQChannelCmd = ENABLE,
    };
  NVIC_Init(&NVIC_InitStructure);

  // Start DMA
  DMA_Cmd(sharps.DMA_Channelx, ENABLE);

  // Create interrupt semphr
  vSemaphoreCreateBinary(xSharpsSemaphore);

  // Create the daemon
  xTaskCreate(vSharpsSmoothTask, (const signed char * const)"sharpsd",
              configMINIMAL_STACK_SIZE, NULL, sharpsDaemonPriority_, NULL);
}

void DMA1_Channel1_IRQHandler()
{
  portBASE_TYPE resched = pdFALSE;
  if (DMA_GetFlagStatus(DMA1_FLAG_TC1))
  {
    xSemaphoreGiveFromISR(xSharpsSemaphore, &resched);
    DMA_ClearITPendingBit(DMA1_IT_TC1);
  }
  portEND_SWITCHING_ISR(resched);
}

static void vSharpsSmoothTask(void* pvParameters_)
{
  static int i = 0;
  static uint32_t average[SHARPS_MAX_NUMBER] = {0};

  for (;;)
  {
    xSemaphoreTake(xSharpsSemaphore, portMAX_DELAY);

    for (int j = 0; j < SHARPS_MAX_NUMBER; j++)
      average[j] += ADC_DMA_Buffer[j];

    i++;

    if (i == AVERAGE_NUMBER)
    {
      for (int j = 0; j < SHARPS_MAX_NUMBER; j++)
      {
        ADC_Smoothed[j] = average[j] / AVERAGE_NUMBER;
        average[j] = 0;
      }
      i = 0;
    }
  }
}

uint16_t uSharpsGetValue(int sharp_)
{
  return ADC_Smoothed[sharp_];
}

