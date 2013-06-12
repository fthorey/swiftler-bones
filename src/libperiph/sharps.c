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

#define MAX_VOLT_MILI     3300
#define ADC_MAX_VALUE     4096
#define GET_VOLTAGE_MILI(V)  ((float)V * MAX_VOLT_MILI / (ADC_MAX_VALUE * 1000))

#define AVERAGE_NB 10
#define DMA_BUFFER_SIZE (AVERAGE_NB * SHARPS_NB)

static volatile uint16_t ADC_DMA_Buffer[DMA_BUFFER_SIZE] = {0};

static sharps_t sharps = { .ADCx = ADC1,
                           .ADCs = {{.GPIOx = GPIOA, .GPIO_Pin_x = GPIO_Pin_6, .ADC_Channel_x = ADC_Channel_6},
                                    {.GPIOx = GPIOC, .GPIO_Pin_x = GPIO_Pin_3, .ADC_Channel_x = ADC_Channel_13}},
                           .DMAx = DMA1,
                           .DMA_Channelx = DMA1_Channel1 };

#define TABLE_SIZE 40

static float table_volt[TABLE_SIZE] = { 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9,
                                        1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9,
                                        2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9,
                                        3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9 };

static float table_length[TABLE_SIZE] = { -1, -1, -1, 42.0, 32.5, 27.0, 22.0, 18.5, 16.0, 14.5,
                                            12.5, 11.5, 10.5, 9.5, 8.4, 8.2, 7.9, 7.2, 6.7, 6.2,
                                            6.0, 5.8, 5.6, 5.4, 5.1, 4.7, 4.3, 4, 3.9, 3.7,
                                            3.6, -1, -1, -1, -1, -1, -1, -1, -1, -1 };

static float iSharpsGetValue(int sharp_);

void vSharpsInit()
{
  // Configure DMA clock
  vDmaClockInit(sharps.DMAx);
  // Configure ADC clock
  vAdcClockInit(sharps.ADCx);

  // Default GPIO config
  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin   = 0,
      .GPIO_Mode  = GPIO_Mode_IN_FLOATING,
      .GPIO_Speed = GPIO_Speed_2MHz
    };

  // GPIO config
  for (int i = 0; i < SHARPS_NB; i++)
  {
    vGpioClockInit(sharps.ADCs[i].GPIOx);
    GPIO_InitStructure.GPIO_Pin = sharps.ADCs[i].GPIO_Pin_x;
    GPIO_Init(sharps.ADCs[i].GPIOx, &GPIO_InitStructure);
  }

  // Reset ADC
  ADC_DeInit(sharps.ADCx);
  // Wake up ADC from Power Down mode
  ADC_Cmd(sharps.ADCx, ENABLE);
  // Wait until it stabilizes (tSTAB)
  vWaitUs(1000);

  // Configure ADC
  ADC_InitTypeDef ADC_InitStructure;
  ADC_StructInit(&ADC_InitStructure);
  /* Independent mode (not dual mode) */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  /* Scan mode -> multichannels conversion */
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  /* Continuous mode (start a new conversion as soon as the previous finished */
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  /* No external trigger to start conversion */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  /* Number of channels to be converted */
  ADC_InitStructure.ADC_NbrOfChannel = SHARPS_NB;
  /* Converted data are aligned to the right (0 0 0 0 D11 D10 ... D0) */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_Init(sharps.ADCx, &ADC_InitStructure);

  // Calibrate ADC
  /* Reset ADC calibration */
  ADC_ResetCalibration(sharps.ADCx);
  /* Wait for the end of the reset calibration */
  while (ADC_GetResetCalibrationStatus(sharps.ADCx));
  /* Start ADC calibration */
  ADC_StartCalibration(sharps.ADCx);
  /* Wait for the end of the calibration */
  while (ADC_GetCalibrationStatus(sharps.ADCx));

  // Enable ADC DMA request
  ADC_DMACmd(sharps.ADCx, ENABLE);

  // Configure ADC channels
  /* Set the order and the sample time of each channel */
  for (int i = 0; i < SHARPS_NB; i++)
    ADC_RegularChannelConfig(sharps.ADCx, sharps.ADCs[i].ADC_Channel_x,
                             i + 1, ADC_SampleTime_239Cycles5);


  // Start ADC conversions
  ADC_Cmd(sharps.ADCx, ENABLE);

  // Initialize DMA
  /* Reset DMA */
  DMA_DeInit(sharps.DMA_Channelx);

  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  /* Set DMA peripheral base addr to ADC buffer */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&sharps.ADCx->DR);
  /* Disable the increment of the periphal addr (always ADC buffer DR reg) */
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  /* Set DMA memory base addr to DMA buffer */
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_DMA_Buffer;
  /* Enable the increment of the mem addr */
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  /* Set the peripheral as the src of the DMA */
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  /* Set the number of data to transfer */
  DMA_InitStructure.DMA_BufferSize = DMA_BUFFER_SIZE;
  /* Set the DMA as a circular DMA (so that the DMA transfer never stop) */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  /* ADC buffer register is 16bit wide -> halfword */
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  /* DMA buffer is 16bit wide too -> halfword */
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  /* Set the DMA priority (not important as only ADC1 use DMA1) */
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Disable Mem to Mem (here it's peripheral to mem) */
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  DMA_Init(sharps.DMA_Channelx, &DMA_InitStructure);

  /* Disable DMA IT (not needed) */
  DMA_ITConfig(sharps.DMA_Channelx, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, DISABLE);

  // Start DMA
  DMA_Cmd(sharps.DMA_Channelx, ENABLE);
}

static float iSharpsGetValue(int sharp_)
{
  uint16_t smoothedValue = 0;
  for (int i = sharp_; i < DMA_BUFFER_SIZE; i += SHARPS_NB)
    smoothedValue += ADC_DMA_Buffer[i];
  return GET_VOLTAGE_MILI(smoothedValue / AVERAGE_NB);
}

float iSharpsMeasureDistCm(int sharp_)
{
  float value = iSharpsGetValue(sharp_);

  int index = -1;
  for (int i = 0; i < TABLE_SIZE; i++)
    if (table_volt[i] > value) {
      index = i;
      break;
    }

  return (table_length[index] + table_length[index-1]) / 2;
}
