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

#define SONAR_BAD_VALUE_TRIG (-1)
#define SONAR_BAD_VALUE_ECHO (-2)

// If no obstacle is detected 38ms is returned
#define SONAR_TIMEOUT_MS 38
#define DEFAULT_TIMEOUT_MS 1

// Trigger pulse timer
// Base clock = 72 Mhz
// Base clock / Prescaler = 72 / 1 = 72Mhz -> Tc = 0.014 us
// Period = 2 * Trig_pulse / Tc = 20 / 0.014 = 1452
#define TIM_TRIG_PSC        0                          // -> div clk by 1
#define TIM_TRIG_PERIOD     1451                       // -> count from 0 to 1451
#define TIM_TRIG_PULSE      (TIM_TRIG_PERIOD / 2)      // -> get a pulse of ~ 10us
#define TIM_TRIG_TC_US      (0.014)                    // -> counter period (us)

// Echo pulse timer
// Base clock = 72 Mhz
// Base clock / Prescaler = 72 / 180 = 400 kHz -> Tc = 2.5 us
// Period = 0x10000 -> we can measure time interval
// up to: Tmax = Tc * 0x10000 ~= 163.84ms
// Seems enough as timeout value is 38ms
#define TIM_ECHO_PSC        179       // -> div clk by 45
#define TIM_ECHO_PERIOD     0xffff    // -> count from 0 to 0xffff
#define TIM_ECHO_TC_US      (2.5)       // -> counter period

// Constructor defined Conversion from echo time to distance
#define CONV_CONST_US_CM   58
#define CONV_CONST_US_INCH 148

// Register if triggering or receiving pulse
#define TRIGGER 0
#define ECHO    1
static bool mode;
// Register if begin or end of echo pulse
#define BEGIN 0
#define END   1
static bool capture;
// Store the echo pulse duration
static int value;
// Store the timer values
static uint16_t IC2Value1;
static uint16_t IC2Value2;

// Semphr for communication between IRQ and main
static xSemaphoreHandle xResponseSemphr;

// Private reusable structures
static TIM_TimeBaseInitTypeDef Timer_InitStructure;
static TIM_ICInitTypeDef TIM_ICInitStructure;
static GPIO_InitTypeDef GPIO_InitStructure;

// Set sonar pin and TIM
static sonar_t sonarPin =
{
  .GPIOx = GPIOC,
  .GPIO_Pin_x = GPIO_Pin_7,
  .TIMx = TIM3
};

void vSonarInit()
{
  // Enable sonar pin clock
  vGpioClockInit(sonarPin.GPIOx);
  // Enable sonar pin TIM
  vTimerClockInit(sonarPin.TIMx);

  // Remap sonar pin TIM
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

  // Register sonar timer interrupt
  NVIC_InitTypeDef NVIC_InitStructure =
    {
      .NVIC_IRQChannel = TIM3_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 7,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    };
  NVIC_Init(&NVIC_InitStructure);

  // Create semaphore
  vSemaphoreCreateBinary(xResponseSemphr);
  // And take it
  xSemaphoreTake(xResponseSemphr,
                 (DEFAULT_TIMEOUT_MS) / portTICK_RATE_MS);
}

static void vSendTriggerPulse()
{
  // Disable sonar timer during configuration
  TIM_Cmd(sonarPin.TIMx, DISABLE);

  // Configure sonar pin
  GPIO_InitStructure.GPIO_Pin   = sonarPin.GPIO_Pin_x;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;     // alternate function push pull
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // we want to detect fast transitions
  GPIO_Init(sonarPin.GPIOx, &GPIO_InitStructure);

  // Reset sonar timer
  TIM_DeInit(sonarPin.TIMx);

  // Configure sonar timer
  Timer_InitStructure.TIM_ClockDivision      = TIM_CKD_DIV1;       // Keep default clk (72Mhz)
  Timer_InitStructure.TIM_Prescaler          = TIM_TRIG_PSC;       // Set to trigger prescaler
  Timer_InitStructure.TIM_Period             = TIM_TRIG_PERIOD;    // Set to trigger period
  Timer_InitStructure.TIM_CounterMode        = TIM_CounterMode_Up;  // Counter goes upward
  TIM_TimeBaseInit(sonarPin.TIMx, &Timer_InitStructure);

  // Clear Update flag
  TIM_ClearFlag(sonarPin.TIMx, TIM_FLAG_Update);

  // Configure output channel 2
  TIM_OCInitTypeDef TIM_OCInitStructure =
    {
      .TIM_OCMode           = TIM_OCMode_PWM2,        // PWM1 mode
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

  // Enable capture compare and update event interrupt
  TIM_ITConfig(sonarPin.TIMx, TIM_IT_CC2 | TIM_IT_Update, ENABLE);

  // Enable sonar timer
  TIM_Cmd(sonarPin.TIMx, ENABLE);
}

void vSetEchoMode()
{
  // Disable sonar timer during configuration
  TIM_Cmd(sonarPin.TIMx, DISABLE);

  // Configure sonar pin
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;     // alternate function input floating
  GPIO_Init(sonarPin.GPIOx, &GPIO_InitStructure);

  // Configure sonar timer
  Timer_InitStructure.TIM_Prescaler          = TIM_ECHO_PSC;       // Set to echo prescaler
  Timer_InitStructure.TIM_Period             = TIM_ECHO_PERIOD;    // Set to echo period
  TIM_TimeBaseInit(sonarPin.TIMx, &Timer_InitStructure);

  // Clear Update flag
  TIM_ClearFlag(sonarPin.TIMx, TIM_FLAG_Update);

  // Configure Input channel 2
  TIM_ICInitStructure.TIM_Channel          = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity       = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection      = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler      = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter         = 0x0;
  TIM_ICInit(sonarPin.TIMx, &TIM_ICInitStructure);

  // Enable sonar timer
  TIM_Cmd(sonarPin.TIMx, ENABLE);

  /* Reset the flags */
  sonarPin.TIMx->SR = 0;
}

void TIM3_IRQHandler()
{
  portBASE_TYPE reschedNeeded = pdFALSE;

  if (TIM_GetITStatus(sonarPin.TIMx, TIM_IT_Update)) {
    // Trigger mode
    if (mode == TRIGGER) {
      // Switch to echo mode
      mode = ECHO;
      // Trigger pulse end: release semphr
      xSemaphoreGiveFromISR(xResponseSemphr, &reschedNeeded);
    }
    // Clear flag
    TIM_ClearITPendingBit(sonarPin.TIMx, TIM_IT_Update);
  }

  else if (TIM_GetITStatus(sonarPin.TIMx, TIM_IT_CC2)) {
    // Echo mode
    if (mode == ECHO) {
      // Start capture
      if (capture == BEGIN) {
        IC2Value1 = TIM_GetCapture2(sonarPin.TIMx);
        // Toggle timer Input channel polarity
        TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
        TIM_ICInit(sonarPin.TIMx, &TIM_ICInitStructure);
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

        // Echo pulse end: release semphr
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

  if (!xSemaphoreTake(xResponseSemphr,
                      (SONAR_TIMEOUT_MS) / portTICK_RATE_MS))
    return SONAR_BAD_VALUE_TRIG;

  // Reset capture value
  capture = BEGIN;

  // Set timer in echo mode
  vSetEchoMode();

  // Wait for the echo pulse end
  if (!xSemaphoreTake(xResponseSemphr,
                      (SONAR_TIMEOUT_MS) / portTICK_RATE_MS))
    return SONAR_BAD_VALUE_ECHO;

  return value * TIM_ECHO_TC_US / CONV_CONST_US_CM;
}
