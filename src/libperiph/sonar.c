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
// Base clock / Prescaler = 72 / 1 = 72 MHz -> Tc = 0.014 us
// Period = 2 * Trig_pulse / Tc = 10 / 0.014 = 1452
#define TIM_TRIG_PSC        8                     // -> div clk by 9
#define TIM_TRIG_PERIOD     1451                  // -> count from 0 to 1451
#define TIM_TRIG_PULSE      (TIM_TRIG_PERIOD / 2) // -> get a pulse of ~ 10us
#define TIM_TRIG_TC_US      (0.014)               // -> counter period (us)

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

// Register if triggering or receiving pulse
enum {TRIGGER, ECHO};
static bool mode;
// Register if begin or end of echo pulse
enum {BEGIN, END};
static bool capture;
// Store the echo pulse duration
static int value;
// Store the timer values
static uint16_t IC2Value1;
static uint16_t IC2Value2;

static xSemaphoreHandle xResponseSemphr;

static sonar_t sonarPin =
{
  .GPIOx = GPIOB,
  .GPIO_Pin_x = GPIO_Pin_5,
  .TIMx = TIM3
};

static void vSendTriggerPulse()
{
  // Disable sonar timer during configuration
  TIM_Cmd(sonarPin.TIMx, DISABLE);

  // Reset sonar timer
  TIM_DeInit(sonarPin.TIMx);

  // Configure sonar timer
  TIM_TimeBaseInitTypeDef Timer_InitStructure =
    {
      .TIM_ClockDivision      = TIM_CKD_DIV1,       // Keep default clk (72Mhz)
      .TIM_Prescaler          = TIM_TRIG_PSC,       // Set to trigger prescaler
      .TIM_Period             = TIM_TRIG_PERIOD,    // Set to trigger period
      .TIM_CounterMode        = TIM_CounterMode_Up  // Counter goes upward
    };
  TIM_TimeBaseInit(sonarPin.TIMx, &Timer_InitStructure);
  // Clear Update flag
  TIM_ClearFlag(sonarPin.TIMx, TIM_FLAG_Update);

  // Configure output channel 2
  TIM_OCInitTypeDef TIM_OCInitStructure =
    {
      .TIM_OCMode           = TIM_OCMode_PWM1,        // PWM1 mode
      .TIM_OutputState      = TIM_OutputState_Enable, // Output compare state enable
      .TIM_Pulse            = TIM_TRIG_PULSE,         // pulse duration (capture compare register value)
      .TIM_OCPolarity       = TIM_OCPolarity_High     // Generate a 0->1 transition when triggering
    };
  TIM_OC2Init(sonarPin.TIMx, &TIM_OCInitStructure);

  // Disable output compare register preload
  TIM_OC2PreloadConfig(sonarPin.TIMx, TIM_OCPreload_Disable);
  // Disable autoreload register preload
  TIM_ARRPreloadConfig(sonarPin.TIMx, DISABLE);

  // Send only one pulse on the sonar pin
  TIM_SelectOnePulseMode(sonarPin.TIMx, TIM_OPMode_Single);

  // Enable capture compare interrupt
  TIM_ITConfig(sonarPin.TIMx, TIM_IT_CC2, ENABLE);

  // Enable sonar timer
  TIM_Cmd(sonarPin.TIMx, ENABLE);
}

void vSetEchoMode()
{
  // Disable sonar timer during configuration
  TIM_Cmd(sonarPin.TIMx, DISABLE);

  // Reset sonar timer
  TIM_DeInit(sonarPin.TIMx);

  // Configure sonar timer
  TIM_TimeBaseInitTypeDef Timer_InitStructure =
    {
      .TIM_ClockDivision      = TIM_CKD_DIV1,       // Keep default clk (72Mhz)
      .TIM_Prescaler          = TIM_ECHO_PSC,       // Set to trigger prescaler
      .TIM_Period             = TIM_ECHO_PERIOD,    // Set to trigger period
      .TIM_CounterMode        = TIM_CounterMode_Up  // Counter goes upward
    };
  TIM_TimeBaseInit(sonarPin.TIMx, &Timer_InitStructure);
  // Clear Update flag
  TIM_ClearFlag(sonarPin.TIMx, TIM_FLAG_Update);


  // Configure Input channel 2
  TIM_ICInitTypeDef TIM_ICInitStructure =
    {
      .TIM_Channel          = TIM_Channel_2,
      .TIM_ICPolarity       = TIM_ICPolarity_BothEdge,
      .TIM_ICSelection      = TIM_ICSelection_DirectTI,
      .TIM_ICPrescaler      = TIM_ICPSC_DIV1,
      .TIM_ICFilter         = 0x0
    };
  TIM_ICInit(sonarPin.TIMx, &TIM_ICInitStructure);

  // Enable capture compare interrupt
  TIM_ITConfig(sonarPin.TIMx, TIM_IT_CC2, ENABLE);

  // Enable sonar timer
  TIM_Cmd(sonarPin.TIMx, ENABLE);

  /* Reset the flags */
  sonarPin.TIMx->SR = 0;
}

void vSonarInit()
{
  // Enable sonar pin clock
  vGpioClockInit(sonarPin.GPIOx);
  // Enable sonar pin TIM
  vTimerClockInit(sonarPin.TIMx);

  // Configure sonar pin
  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin   = sonarPin.GPIO_Pin_x,
      .GPIO_Mode  = GPIO_Mode_AF_PP, // alternate function push pull
      .GPIO_Speed = GPIO_Speed_50MHz // we want to detect fast transitions
    };
  GPIO_Init(sonarPin.GPIOx, &GPIO_InitStructure);

  // Register sonar timer interrupt
  NVIC_InitTypeDef NVIC_InitStructure =
    {
      .NVIC_IRQChannel = TIM3_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 7,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    };
  NVIC_Init(&NVIC_InitStructure);
}

void TIM3_IRQHandler()
{
  portBASE_TYPE reschedNeeded = pdFALSE;

  if (TIM_GetITStatus(sonarPin.TIMx, TIM_IT_CC2)) {

    // Trigger mode
    if (mode == TRIGGER) {
      // Switch to echo mode
      xSemaphoreGiveFromISR(xResponseSemphr, &reschedNeeded);
      mode = ECHO;
    }

    // Echo mode
    else if (mode == ECHO) {
      // Start capture
      if (capture == BEGIN) {
          IC2Value1 = TIM_GetCapture2(sonarPin.TIMx);
          capture = END;
      }
      // End capture
      else if (capture == END) {
        IC2Value2 = TIM_GetCapture2(sonarPin.TIMx);
        if (IC2Value2 > IC2Value1) {
          value = (IC2Value2 - IC2Value1) - 1;
        }
        else {
          value = ((0xFFFF - IC2Value1) + IC2Value2) - 1;
        }
        xSemaphoreGiveFromISR(xResponseSemphr, &reschedNeeded);
      }
    }

    // Clear flag
    TIM_ClearITPendingBit(sonarPin.TIMx, TIM_IT_CC2);

  }

  portEND_SWITCHING_ISR(reschedNeeded);
}

int iSonarMeasureDistCm()
{
  // Send sonar trigger pulse
  mode = TRIGGER;
  vSendTriggerPulse();

  // Wait for the trigger pulse end
  if (!xSemaphoreTake(xResponseSemphr,
                      (SONAR_TIMEOUT_MS + 2) / portTICK_RATE_MS))
    return SONAR_BAD_VALUE;

  // Reset capture value
  capture = BEGIN;

  // Set timer in echo mode
  vSetEchoMode();

  // Wait for the echo pulse end
  if (!xSemaphoreTake(xResponseSemphr,
                      (SONAR_TIMEOUT_MS + 2) / portTICK_RATE_MS))
    return SONAR_BAD_VALUE;

  return value * TIM_ECHO_TC_US / CONV_CONST_US_CM;
}