#include "libperiph/leds.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "task.h"
#include "libperiph/hardware.h"

# define LEDS_CNT 1

typedef struct
{
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin_x;
} led_t;

static void prvFlashLEDTask(void* pvParameters_);

static led_t leds[LEDS_CNT] = {{.GPIOx = GPIOA, .GPIO_Pin_x = GPIO_Pin_5}}; // LED_GREEN

void vLedsInit(unsigned portBASE_TYPE ledDaemonPriority_)
{
  for (int i = 0; i < LEDS_CNT; i++) {
    vGpioClockInit(leds[i].GPIOx);

    GPIO_InitTypeDef init =
      {
        .GPIO_Pin   = leds[i].GPIO_Pin_x,
        .GPIO_Speed = GPIO_Speed_2MHz,
        .GPIO_Mode  = GPIO_Mode_Out_PP,
      };

    GPIO_Init(leds[i].GPIOx, &init);
    GPIO_ResetBits(leds[i].GPIOx, leds[i].GPIO_Pin_x);
  }

  xTaskCreate(prvFlashLEDTask,
              (signed portCHAR*)"Flash LED",
              configMINIMAL_STACK_SIZE, NULL,
              ledDaemonPriority_, NULL);

}

static void prvFlashLEDTask(void* pvParameters_)
{
  for (;;) {
    vLedToggle(LED_GREEN);
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}

void vLedOff(enum eLED led_)
{
  GPIO_ResetBits(leds[led_].GPIOx, leds[led_].GPIO_Pin_x);
}

void vLedOn(enum eLED led_)
{
  GPIO_SetBits(leds[led_].GPIOx, leds[led_].GPIO_Pin_x);
}

void vLedToggle(enum eLED led_)
{
  if (GPIO_ReadOutputDataBit(leds[led_].GPIOx, leds[led_].GPIO_Pin_x))
    vLedOff(led_);
  else
    vLedOn(led_);
}
