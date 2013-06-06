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
#define DEFAULT_TIMEOUT_MS 1

// Trigger pulse timer
// Base clock = 72 Mhz
// Base clock / Prescaler = 72 / 0x10000 = 1kHz -> Tc = 1 ms
// Period = 2 * Trig_pulse / Tc = 20 / 0.014 = 1452
#define TIM_TRIG_PSC        0xFFFF                     // -> div clk by 1
#define TIM_TRIG_PERIOD     2000                       // -> count from 0 to 1451
#define TIM_TRIG_PULSE      (TIM_TRIG_PERIOD / 2)      // -> get a pulse of ~ 10us
#define TIM_TRIG_TC_US      (0.014)                    // -> counter period (us)

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
#define TRIGGER 0
#define ECHO    1
static bool mode;

static xSemaphoreHandle xResponseSemphr;

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

  // Configure sonar pin
  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin   = sonarPin.GPIO_Pin_x,
      .GPIO_Mode  = GPIO_Mode_AF_PP, // alternate function push pull
      .GPIO_Speed = GPIO_Speed_50MHz // we want to detect fast transitions
    };
  GPIO_Init(sonarPin.GPIOx, &GPIO_InitStructure);

  // Configure test pin
  GPIO_InitTypeDef GPIO_InitStructure2 =
    {
      .GPIO_Pin   = GPIO_Pin_5,
      .GPIO_Mode  = GPIO_Mode_Out_PP, // alternate function push pull
      .GPIO_Speed = GPIO_Speed_2MHz // we want to detect fast transitions
    };
  GPIO_Init(GPIOC, &GPIO_InitStructure2);

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

  // Enable capture compare interrupt
  TIM_ITConfig(sonarPin.TIMx, TIM_IT_CC2 | TIM_IT_Update, ENABLE);

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

  // Enable capture compare interrupt
  TIM_ITConfig(sonarPin.TIMx, TIM_IT_CC2, ENABLE);

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
      // Switch on test pin
      GPIO_ResetBits(GPIOC, GPIO_Pin_5);
      // Switch to echo mode
      mode = ECHO;
      xSemaphoreGiveFromISR(xResponseSemphr, &reschedNeeded);
    }
    // Clear flag
    TIM_ClearITPendingBit(sonarPin.TIMx, TIM_IT_Update);
  }

  else if (TIM_GetITStatus(sonarPin.TIMx, TIM_IT_CC2)) {
    // Switch off test pin
    GPIO_SetBits(GPIOC, GPIO_Pin_5);
    // Clear flag
    TIM_ClearITPendingBit(sonarPin.TIMx, TIM_IT_CC2);
  }

  portEND_SWITCHING_ISR(reschedNeeded);
}

int iSonarMeasureDistCm()
{
  // Switch on test pin
  GPIO_ResetBits(GPIOC, GPIO_Pin_5);

  // Send sonar trigger pulse
  mode = TRIGGER;
  vSendTriggerPulse();

  if (!xSemaphoreTake(xResponseSemphr,
                      (SONAR_TIMEOUT_MS) / portTICK_RATE_MS))
    return -1;

  return 0;
}
