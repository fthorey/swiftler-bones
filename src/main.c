#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "libglobal/interpreter.h"
#include "libglobal/strutils.h"

#include "libperiph/hardware.h"
#include "libperiph/uart.h"
#include "libperiph/leds.h"
#include "libperiph/motors.h"
#include "libperiph/sonar.h"
#include "libperiph/sharps.h"

static bool bMotorsEnable   = ENABLE;

void process_motor_cmd(char* str);
void process_sonar_cmd(char* str);

int main(void)
{
  // HARDWARE
  vHardwareInit();

  // UART
  vUartInit();

  // LEDS
  vLedsInit(tskIDLE_PRIORITY + 3);

  // SONAR
  vSonarInit();

  // SHARPS
  /* vSharpsInit(tskIDLE_PRIORITY + 3); */

  // MOTORS
  vMotorsInit(tskIDLE_PRIORITY + 3);

  // INTERPRETER
  token_t tokens[2];
  tokens[0].command = 'm';
  tokens[0].handler = &process_motor_cmd;
  tokens[1].command = 's';
  tokens[1].handler = &process_sonar_cmd;
  vInterpreterInit("woggle", &tokens[0], 2, tskIDLE_PRIORITY + 4);

  vInterpreterStart();

  if (bMotorsEnable)
    vMotorsEnable();

  vTaskStartScheduler();

  return 0;
}

void process_sonar_cmd(char* str)
{
  char buffer[32];
  itoa(iSonarMeasureDistCm(), buffer);
  vUartPuts(buffer);
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
      vUartPuts("stop motors");
      vUartPuts("\r\n");
      bMotorsEnable = DISABLE;
    }
    else
    {
      vMotorsEnable();
      vUartPuts("start motors");
      vUartPuts("\r\n");
      bMotorsEnable = ENABLE;
    }
  }
  else if (cmd == 'l')
  {
    value = atoi(args);
    vSetMotorLeftCommand(value);
    vUartPuts("setting LEFT motor speed: ");
    itoa(value, buffer);
    vUartPuts(buffer);
    vUartPuts("%\r\n");
  }
  else if (cmd == 'r')
  {
    value = atoi(args);
    vSetMotorRightCommand(value);
    vUartPuts("setting RIGHT motor speed: ");
    itoa(value, buffer);
    vUartPuts(buffer);
    vUartPuts("%\r\n");
  }
  else if (cmd == 'b')
  {
    int sep = 1;
    while (args[sep++] != ':');
    value = atoi_eol(args, ':');
    vSetMotorLeftCommand(value);
    vUartPuts("setting LEFT motor speed: ");
    itoa(value, buffer);
    vUartPuts(buffer);
    vUartPuts("%\r\n");

    value = atoi(args + sep);
    vSetMotorRightCommand(value);
    vUartPuts("setting RIGHT motor speed: ");
    itoa(value, buffer);
    vUartPuts(buffer);
    vUartPuts("%\r\n");
  }
  else
  {
    vUartPuts("error: undefined subcommand '");
    vUartPutc(cmd);
    vUartPuts("' for motor command\r\n");
  }
}

