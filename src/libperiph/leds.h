#ifndef LIBPERIPH_LEDS_H
# define LIBPERIPH_LEDS_H

#include "FreeRTOS.h"

enum eLED {
  LED_GREEN  = 0,
  LED_YELLOW = 1
};

void vLedsInit(unsigned portBASE_TYPE ledDaemonPriority_);
void vLedOn(enum eLED led_);
void vLedOff(enum eLED led_);
void vLedToggle(enum eLED led_);

#endif
