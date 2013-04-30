#ifndef INTERPRETER_H
# define INTERPRETER_H

#include "FreeRTOS.h"

typedef void (*pfunTokenHandle) (char*);

typedef struct
{
  char command;
  pfunTokenHandle handler;
} token_t;

void vInterpreterInit(const char* pr, token_t* tok, int n,
                      unsigned portBASE_TYPE daemon_priority);
void vInterpreterStart();

#endif
