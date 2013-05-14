#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "interpreter.h"
#include "libglobal/strutils.h"
#include "libperiph/uart.h"

static char prompt[32];
static token_t tokens[32];
static int n_tokens;
static unsigned portBASE_TYPE priority;

static void prvInterpreterDaemon(void* pvParameters);

void vInterpreterInit(const char* pr, token_t* tok, int n,
                      unsigned portBASE_TYPE daemon_priority)
{
  memcpy(prompt, pr, strlen(pr) * sizeof (char));
  n_tokens = n;
  for (int i = 0; i < n; i++)
  {
    tokens[i].command = tok[i].command;
    tokens[i].handler = tok[i].handler;
  }
  priority = daemon_priority;
}

void vInterpreterStart()
{
  xTaskCreate(prvInterpreterDaemon,
              (signed portCHAR*)"Interpreter",
              configMINIMAL_STACK_SIZE, NULL,
              priority, NULL);
}

static void prvInterpreterDaemon(void* pvParameters)
{
  char c;
  char buffer[32];
  char* cmd;
  int abort, size;
  vTaskDelay(1000);
  vUartPuts("\r\n");
  for (;;)
  {
    abort = 0;
    size = 0;

    vUartPuts(prompt);
    vUartPuts(" # ");

    buffer[0] = 0;
    while ((c = cUartGetc()))
    {
      if (size == 32)
      {
        abort = 1;
        vUartPuts("\r\nerror: command too long...\r\n");
        break;
      }

      if (c == '\r' || c == '\n')
      {
        vUartPuts("\r\n");
        buffer[size] = 0;
        break;
      }
      else if (c == 0x03)
      {
        abort = 1;
        vUartPuts("\r\n");
        buffer[size] = 0;
        vUartPuts(buffer);
        vUartPuts(": abort\r\n");
        break;
      }
      else if (c == 0x08 || c == 0x7f)
      {
        if (size > 0)
        {
          size--;
          vUartPutc(c);
          vUartPutc(' ');
          vUartPutc(c);
        }
        continue;
      }
      else if (!is_letter(c) && !is_number(c)
               && !is_space(c) && c != '-'
               && c != ':'     && c != '.')
        continue;

      vUartPutc(c);
      buffer[size++] = c;
    }

    if (abort)
      continue;

    cmd = trim_in_place(buffer);

    // Skip empty command
    if (cmd[0] == 0)
      continue;

    abort = 1;
    for (int i = 0; i < n_tokens; i++)
    {
      if (cmd[0] == tokens[i].command)
      {
        (*tokens[i].handler)(cmd + 1);
        abort = 0;
        vUartPuts("\r\n");
        break;
      }
    }

    if (abort)
    {
      vUartPuts("error: undefined command '");
      vUartPutc(cmd[0]);
      vUartPuts("\r\n");
    }
  }
}
