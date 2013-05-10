#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "libglobal/interpreter.h"
#include "libglobal/strutils.h"

#include "libperiph/hardware.h"
#include "libperiph/uart.h"

#include "libperiph/leds.h"

static void prvFlashLEDTask(void* pvParameters);

void cmd_received(char* cmd);

int main(void)
{
  hardware_init();
  led_init();
  uart_init();

  token_t token;
  token.command = 'c';
  token.handler = &cmd_received;
  vInterpreterInit("woggle", &token, 1, tskIDLE_PRIORITY + 4);

  xTaskCreate(prvFlashLEDTask,
              (signed portCHAR*)"Flash LED",
              configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 1, NULL);

  vInterpreterStart();

  vTaskStartScheduler();

  return 0;
}

static void prvFlashLEDTask(void* pvParameters)
{
  for (;;) {
    led_toggle(LED_GREEN);
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}

void cmd_received(char* str)
{
  uart_puts("cmd_received");
}
