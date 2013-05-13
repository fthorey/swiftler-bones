#ifndef MOTORS_H
# define MOTORS_H

#include <stdint.h>

#include "FreeRTOS.h"

void vMotorsInit(unsigned portBASE_TYPE motorsDaemonPriority_);

void vMotorsEnable();
void vMotorsDisable();

void vSetMotorsCommand(int16_t left_, int16_t right_);
void vSetMotorLeftCommand(int16_t left_);
void vSetMotorRightCommand(int16_t right_);

uint16_t uGetMotorLeftCommand();
uint16_t uGetMotorRightCommand();

#endif
