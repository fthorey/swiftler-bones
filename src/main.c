#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "libglobal/interpreter.h"
#include "libglobal/strutils.h"

#include "libperiph/hardware.h"
#include "libperiph/uart.h"

#include "libperiph/leds.h"

static void prvFlashLEDTask(void* pvParameters);

void process_motor_cmd(char* cmd);
void process_control_cmd(char* cmd);
void process_debug_cmd(char* cmd);

int main(void)
{
  hardware_init();
  led_init();
  uart_init();

  token_t tokens[3];
  tokens[0].command = 'm';
  tokens[0].handler = &process_motor_cmd;
  tokens[1].command = 'c';
  tokens[1].handler = &process_control_cmd;
  tokens[2].command = 'd';
  tokens[2].handler = &process_debug_cmd;
  vInterpreterInit("woggle", &tokens[0], 3, tskIDLE_PRIORITY + 4);

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
    led_toggle(LED_YELLOW);
    vTaskDelay(500 / portTICK_RATE_MS);
    led_toggle(LED_YELLOW);
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}

void process_motor_cmd(char* str)
{
  uart_puts("process_motor_cmd received");
}


void process_control_cmd(char* str)
{
  uart_puts("process_control_cmd received");
}

void process_debug_cmd(char* cmd)
{
  uart_puts("process_debug_cmd received");
}


