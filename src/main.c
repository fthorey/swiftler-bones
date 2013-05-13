#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "libglobal/interpreter.h"
#include "libglobal/strutils.h"

#include "libperiph/hardware.h"
#include "libperiph/uart.h"
#include "libperiph/leds.h"
#include "libperiph/motors.h"

static bool bMotorsEnable   = ENABLE;

void process_motor_cmd(char* str);

int main(void)
{
  // HARDWARE
  hardware_init();

  // UART
  uart_init();

  // LEDS
  vLedsInit(tskIDLE_PRIORITY + 1);

  // MOTORS
  vMotorsInit(tskIDLE_PRIORITY + 3);

  // SERIAL CONSOLE
  token_t token;
  token.command = 'm';
  token.handler = &process_motor_cmd;
  vInterpreterInit("woggle", &token, 1, tskIDLE_PRIORITY + 4);

  vInterpreterStart();

  if (bMotorsEnable)
    vMotorsEnable();

  vTaskStartScheduler();

  return 0;
}

void process_motor_cmd(char* str)
{
  int value;
  char buffer[32];
  char cmd = str[0];
  char* args = trim_in_place(str + 1);

  if (cmd == 's') // start/stop
  {
    if (bMotorsEnable)
    {
      vMotorsDisable();
      uart_puts("stop motors");
      uart_puts("\r\n");
      bMotorsEnable = DISABLE;
    }
    else
    {
      vMotorsEnable();
      uart_puts("start motors");
      uart_puts("\r\n");
      bMotorsEnable = ENABLE;
    }
  }
  else if (cmd == 'l')
  {
    value = atoi(args);
    vSetMotorLeftCommand(value);
    uart_puts("setting LEFT motor speed: ");
    itoa(value, buffer);
    uart_puts(buffer);
    uart_puts("%\r\n");
  }
  else if (cmd == 'r')
  {
    value = atoi(args);
    vSetMotorRightCommand(value);
    uart_puts("setting RIGHT motor speed: ");
    itoa(value, buffer);
    uart_puts(buffer);
    uart_puts("%\r\n");
  }
  else if (cmd == 'b')
  {
    int sep = 1;
    while (args[sep++] != ':');
    value = atoi_eol(args, ':');
    vSetMotorLeftCommand(value);
    uart_puts("setting LEFT motor speed: ");
    itoa(value, buffer);
    uart_puts(buffer);
    uart_puts("%\r\n");

    value = atoi(args + sep);
    vSetMotorRightCommand(value);
    uart_puts("setting RIGHT motor speed: ");
    itoa(value, buffer);
    uart_puts(buffer);
    uart_puts("%\r\n");
  }
  else
  {
    uart_puts("error: undefined subcommand '");
    uart_putc(cmd);
    uart_puts("' for motor command\r\n");
  }
}

