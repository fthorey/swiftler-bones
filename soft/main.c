#include "FreeRTOS.h"
#include "task.h"

#include "libperiph/hardware.h"

#include "leds.h"

static void prvFlashLEDTask(void* pvParameters);

int main(void)
{
  hardware_init();
  vLEDsInit();

  vLEDOn(COLOR_LED_GREEN);

  xTaskCreate(prvFlashLEDTask,
              (signed portCHAR*)"Flash LED",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 1, NULL);

  vTaskStartScheduler();

  return 0;
}

static void prvFlashLEDTask(void* pvParameters)
{
  for (;;) {
    vLEDToggle(COLOR_LED_GREEN);
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}
