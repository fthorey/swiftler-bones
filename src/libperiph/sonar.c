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

// If no obstacle is detected 38ms is returned
#define SONAR_TIMEOUT_MS 38

// Trigger pulse timer
// Base clock = 72 Mhz
// Base clock / Prescaler = 72 / 9 = 8 MHz -> Tc = 0.125 us
// Period = Trig_pulse / Tc = 10 / 0.125 = 80
#define TIM_TRIG_PSC        8        // -> div clk by 9
#define TIM_TRIG_PERIOD     79       // -> count from 0 to 79
#define TIM_TRIG_TC_US      (0.125)  // -> counter period (us)

// Echo pulse timer
// Base clock = 72 Mhz
// Base clock / Prescaler = 72 / 45 = 1.6 MHz -> Tc = 0.625 us
// Period = 0x10000 -> we can measure time interval
// up to: Tmax = Tc * 0x10000 ~= 41ms
// Seems enough as timeout value is 38ms
#define TIM_ECHO_PSC        44       // -> div clk by 45
#define TIM_ECHO_PERIOD     0xffff   // -> count from 0 to 0xffff
#define TIM_ECHO_TC_US      (0.0625) // -> counter period

// Constructor defined Conversion from echo time to distance
#define CONV_CONST_US_CM   58
#define CONV_CONST_US_INCH 148

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
  vTimerClockInit(TIM3);

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

  // Configure trigger pulse TIM
  // Reset TIM
  TIM_DeInit(TIM3);
  // Disable by Software Update Event for now
  TIM_UpdateDisableConfig(TIM3, ENABLE);
  TIM_TimeBaseInitTypeDef Timer_InitStructure =
    {
      .TIM_ClockDivision      = TIM_CKD_DIV1,
      .TIM_Prescaler          = TIM_ECHO_PSC,
      .TIM_Period             = TIM_ECHO_PERIOD,
      .TIM_CounterMode        = TIM_CounterMode_Up
    };
  TIM_TimeBaseInit(TIM3, &Timer_InitStructure);
  // Enables TIM peripheral Preload register on ARR
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  // Enable TIM Update Event Interrupt
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  // Disable TIM
  TIM_Cmd(TIM3, DISABLE);

  //  Register trigger pulse TIM interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Configure echo back TIM
  // Reset TIM
  TIM_DeInit(TIM1);
  Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  Timer_InitStructure.TIM_Prescaler     = TIM_ECHO_PSC;
  Timer_InitStructure.TIM_Period        = TIM_ECHO_PERIOD;
  Timer_InitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
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

void TIM3_IRQHandler()
{
 portBASE_TYPE reschedNeeded = pdFALSE;

 vGpioClockInit(GPIOC);

 if (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update))
   {
     // 10us has elapsed:
     // Stop sending trigger pulse
     GPIO_WriteBit(sonarPin.GPIOx, sonarPin.GPIO_Pin_x, Bit_RESET);
     // Stop trigger TIM
     // Disable by software Update Event
     TIM_UpdateDisableConfig(TIM3, ENABLE);
     TIM_Cmd(TIM3, DISABLE);
     // Set sonarPin in receive mode
     vSetPinInReceiveMode(sonarPin);
     // Clear Update flag
     TIM_ClearFlag(TIM3, TIM_FLAG_Update);
   }
 portEND_SWITCHING_ISR(reschedNeeded);
}

void EXTI2_IRQHandler()
{
  portBASE_TYPE reschedNeeded = pdFALSE;
  if (EXTI_GetFlagStatus(EXTI_Line2))
  {
    // Echo end
    if (enabled)
    {
      // Get echo TIM value
      value = TIM_GetCounter(TIM1);
      TIM_Cmd(TIM1, DISABLE);
      xSemaphoreGiveFromISR(xResponseSemphr, &reschedNeeded);
      enabled = FALSE;
    }
    // Echo start
    else
    {
      // Start echo TIM
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
  int time_us;
  // Reset echo TIM
  TIM_SetCounter(TIM1, 0);
  enabled = FALSE;
  // Reset trigger TIM
  TIM_SetCounter(TIM3, 0);

  // Set sonarPin in send mode
  vSetPinInSendMode(sonarPin);

  // Start trigger TIM
  TIM_Cmd(TIM3, ENABLE);
  // Enable by software Update Event
  TIM_UpdateDisableConfig(TIM3, DISABLE);
  // Start Sending trigger pulse
  GPIO_WriteBit(sonarPin.GPIOx, sonarPin.GPIO_Pin_x, Bit_SET);

  // Wait for the echo
  // If the echo takes too much time -> stop waiting
  if (!xSemaphoreTake(xResponseSemphr,
                      (SONAR_TIMEOUT_MS + 2) / portTICK_RATE_MS))
    return SONAR_BAD_VALUE;

  // Calculate interval time
  // time_us = value * Tc
  time_us = value * (TIM_ECHO_TC_US);

  return time_us / CONV_CONST_US_CM;
}
