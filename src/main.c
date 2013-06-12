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
#include "libperiph/i2c.h"

#define CONSOLE_TOKEN_NB 4

static bool bMotorsEnable   = ENABLE;

void process_motor_cmd(char* str);
void process_sonar_cmd(char* str);
void process_sharps_cmd(char* str);
void process_sensors_cmd(char* str);

int main(void)
{
  // Hardware
  vHardwareInit();
  // Uart
  vUartInit();
  // I2C
  vI2CInit();
  // Leds
  vLedsInit(tskIDLE_PRIORITY + 3);
  // Sonar
  vSonarInit(tskIDLE_PRIORITY + 3);
  // Sharps
  vSharpsInit();
  // Motors
  vMotorsInit(tskIDLE_PRIORITY + 3);

  // Interpreter
  token_t tokens[CONSOLE_TOKEN_NB];
  tokens[0].command = 'm';
  tokens[0].handler = &process_motor_cmd;
  tokens[1].command = 's';
  tokens[1].handler = &process_sonar_cmd;
  tokens[2].command = 'i';
  tokens[2].handler = &process_sharps_cmd;
  tokens[3].command = 'a';
  tokens[3].handler = &process_sensors_cmd;
  vInterpreterInit("woggle", &tokens[0], CONSOLE_TOKEN_NB, tskIDLE_PRIORITY + 4);
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


void process_sharps_cmd(char* str)
{
  char buffer[32];
  fltoa(iSharpsMeasureDistCm(SHARP_LEFT), buffer);
  vUartPuts(buffer);
  vUartPutc('\t');
  fltoa(iSharpsMeasureDistCm(SHARP_RIGHT), buffer);
  vUartPuts(buffer);
}

void process_sensors_cmd(char* str)
{
  char buffer[32];
  // Left sharp
  fltoa(iSharpsMeasureDistCm(SHARP_LEFT), buffer);
  vUartPuts(buffer);
  vUartPutc('\t');
  // Central sonar
  itoa(iSonarMeasureDistCm(), buffer);
  vUartPuts(buffer);
  vUartPutc('\t');
  // Right sharp
  fltoa(iSharpsMeasureDistCm(SHARP_RIGHT), buffer);
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

