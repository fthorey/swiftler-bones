#ifndef __MOTORDRIVER_H__
#define __MOTORDRIVER_H__
#include <Arduino.h>
/******Pins definitions*************/
#define MOTORSHIELD_IN1	8
#define MOTORSHIELD_IN2	11
#define MOTORSHIELD_IN3	12
#define MOTORSHIELD_IN4	13
#define SPEEDPIN_A		9
#define SPEEDPIN_B		10
/**************Motor ID**********************/
#define MOTORA 			0
#define MOTORB 			1

#define MOTOR_POSITION_LEFT  0
#define MOTOR_POSITION_RIGHT 1
#define MOTOR_CLOCKWISE      0
#define MOTOR_ANTICLOCKWISE  1

#define USE_DC_MOTOR		0

struct MotorStruct
{
	int8_t speed;
	uint8_t direction;
	uint8_t position;
};
/**Class for Motor Shield**/
class MotorDriver
{
	MotorStruct motorA;
	MotorStruct motorB;
public:
	void init();
	void configure(uint8_t position, uint8_t motorID);
	void setSpeed(int8_t speed, uint8_t motorID);
	void setDirection(uint8_t direction, uint8_t motorID);
	void rotate(uint8_t direction, uint8_t motor_position);
	void rotateWithID(uint8_t direction, uint8_t motorID);
	void goForward();
	void goBackward();
	void goLeft();
	void goRight();
	void stop();
	void stop(uint8_t motorID);
};
extern MotorDriver motordriver;

#endif
