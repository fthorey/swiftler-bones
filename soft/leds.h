#ifndef LEDS_H
# define LEDS_H

enum eColorLED {
  COLOR_LED_RED   = 0,
  COLOR_LED_GREEN = 1,
  COLOR_LED_BLUE  = 2,
};

enum eColor {
  BLACK   = 0b000,
  RED     = 0b001,
  GREEN   = 0b010,
  BLUE    = 0b100,
  YELLOW  = 0b011,
  MAGENTA = 0b101,
  CYAN    = 0b110,
  WHITE   = 0b111,
};

void vLEDsInit();
void vLEDOn(enum eColorLED eLED);
void vLEDOff(enum eColorLED eLED);
void vLEDToggle(enum eColorLED eLED);
void vLEDSetColor(enum eColor eLEDColor);

#endif
