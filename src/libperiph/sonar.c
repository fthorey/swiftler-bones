#include "sonar.h"

#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "libglobal/strutils.h"

#include "libperiph/uart.h"
#include "libperiph/hardware.h"

#define SONAR_BAD_VALUE (-1)
#define SONAR_TIMEOUT_MS 30

static xSemaphoreHandle xResponseSemphr;

static sonar_t sonarPin =
{
  .GPIOx = GPIOC,
  .GPIO_Pin_x = GPIO_Pin_2,
  .EXTI_Line = EXTI_Line2,
};

static bool enabled = FALSE;
static int value;

static void vSetPinInReceiveMode(sonar_t pin_);
static void vSetPinInSendMode(sonar_t pin_);

void vSonarInit()
{
  // Enable Clocks
  vGpioClockInit(sonarPin.GPIOx);
  vTimerClockInit(TIM1);

  // Link EXTI line to sonar pin
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);

  // Configure sonarPin in send mode
  vSetPinInSendMode(sonarPin);

  // Register EXTI2 interrupt
  NVIC_InitTypeDef NVIC_InitStructure =
    {
      .NVIC_IRQChannel = EXTI2_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 7,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    };
  NVIC_Init(&NVIC_InitStructure);

  // Configure TIM
  TIM_TimeBaseInitTypeDef Timer_InitStructure =
    {
      .TIM_ClockDivision      = TIM_CKD_DIV1,
      .TIM_Prescaler          = 44,
      .TIM_Period             = 0xffff,
      .TIM_CounterMode        = TIM_CounterMode_Up
    };
  TIM_TimeBaseInit(TIM1, &Timer_InitStructure);

  // Enables TIM peripheral Preload register on ARR
  TIM_ARRPreloadConfig(TIM1, ENABLE);

  // Disable TIM
  TIM_Cmd(TIM1, DISABLE);

  // Create interrupt semaphore
  vSemaphoreCreateBinary(xResponseSemphr);
}

static void vSetPinInReceiveMode(sonar_t pin_)
{
  // Set pin in input
  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin   = pin_.GPIO_Pin_x,
      .GPIO_Mode  = GPIO_Mode_IPD,
      .GPIO_Speed = GPIO_Speed_2MHz
    };
  GPIO_Init(pin_.GPIOx, &GPIO_InitStructure);

  // Enable EXTI line
  EXTI_InitTypeDef EXTI_InitStructure =
    {
      .EXTI_Line = pin_.EXTI_Line,
      .EXTI_Mode = EXTI_Mode_Interrupt,
      .EXTI_Trigger = EXTI_Trigger_Rising_Falling,
      .EXTI_LineCmd = ENABLE
    };
  EXTI_Init(&EXTI_InitStructure);
}

static void vSetPinInSendMode(sonar_t pin_)
{
  // Disable EXTI line
  EXTI_InitTypeDef EXTI_InitStructure =
    {
      .EXTI_Line = pin_.EXTI_Line,
      .EXTI_Mode = EXTI_Mode_Interrupt,
      .EXTI_Trigger = EXTI_Trigger_Rising_Falling,
      .EXTI_LineCmd = DISABLE,
    };
  EXTI_Init(&EXTI_InitStructure);

  // Set pin in output
  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin   = pin_.GPIO_Pin_x,
      .GPIO_Mode  = GPIO_Mode_Out_PP,
      .GPIO_Speed = GPIO_Speed_2MHz
    };
  GPIO_Init(pin_.GPIOx, &GPIO_InitStructure);

  // Reset Pin
  GPIO_WriteBit(pin_.GPIOx, pin_.GPIO_Pin_x, Bit_RESET);
}

void EXTI2_IRQHandler()
{
  portBASE_TYPE reschedNeeded = pdFALSE;
  if (EXTI_GetFlagStatus(EXTI_Line2))
  {
    // Echo end
    if (enabled)
    {
      // Get TIM value
      value = TIM_GetCounter(TIM1);
      TIM_Cmd(TIM1, DISABLE);
      xSemaphoreGiveFromISR(xResponseSemphr, &reschedNeeded);
      enabled = FALSE;
    }
    // Echo start
    else
    {
      // Start TIM
      TIM_Cmd(TIM1, ENABLE);
      enabled = TRUE;
    }
    // Clear interrupt
    EXTI_ClearFlag(EXTI_Line2);
  }
  portEND_SWITCHING_ISR(reschedNeeded);
}

int iSonarMeasureDistCm()
{
  int us;
  TIM_SetCounter(TIM1, 0);
  enabled = FALSE;

  // Set sonarPin in send mode
  vSetPinInSendMode(sonarPin);

  // Send pulse
  GPIO_WriteBit(sonarPin.GPIOx, sonarPin.GPIO_Pin_x, Bit_SET);
  vTaskDelay(1 / portTICK_RATE_MS);
  GPIO_WriteBit(sonarPin.GPIOx, sonarPin.GPIO_Pin_x, Bit_RESET);

  // Set sonarPin in receive mode
  vSetPinInReceiveMode(sonarPin);

  // Wait for the echo
  if (!xSemaphoreTake(xResponseSemphr,
                      (SONAR_TIMEOUT_MS + 2) / portTICK_RATE_MS))
    return SONAR_BAD_VALUE;

  us = value * 40000 / 0xffff;

  if (us >= (SONAR_TIMEOUT_MS - 2) * 1000)
    return SONAR_BAD_VALUE;

  return value;
}
