#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "libperiph/hardware.h"
#include "libperiph/motors.h"

// f = 72MHz / 4 / 1000 = 18 kHz
#define PERIOD          999  // (-> count from 0 to 999)
#define DEFAULT_PSC     3    // (-> div clk by 4)
#define BEEP_PSC        71   // (-> div clk by 72)

#define OFFSET          (PERIOD / 2)
#define LIMIT_VAL       100
#define MAX_DIFF        50

#define COMMAND_TO_PWM(C) ((((C) * OFFSET) / LIMIT_VAL) + OFFSET)

typedef union{
  struct {
    int16_t left;
    int16_t right;
  } motor;
  uint32_t motors;
} motors_command_t;

static volatile motors_command_t targetCommand;
static motors_command_t previousCommand;
static motors_command_t currentCommand;

static void vMotorsApplyCommands(motors_command_t cmd_);
static motors_command_t iMotorsLimitCommands(motors_command_t targ_,
                                             motors_command_t prev_);

static void vMotorsTask(void* pvParameters_);
static void vMotorsReset();

void vMotorsInit(unsigned portBASE_TYPE motorsDaemonPriority_)
{
  // Enable GPIOA &  GPIOC clock
  vGpioClockInit(GPIOA);
  vGpioClockInit(GPIOC);

  // Enable TIM2 clock
  vTimerClockInit(TIM2);

  // Motors PWM: MOTOR1=left, MOTOR2=right ; A and B have opposed polarity
  GPIO_InitTypeDef GPIO_InitStructure1 =
  {
    .GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | // MOTOR2_B, MOTOR2_A
                  GPIO_Pin_2 | GPIO_Pin_3 , // MOTOR1_B, MOTOR1_A
    .GPIO_Mode  = GPIO_Mode_AF_PP,          // alternate function push pull
    .GPIO_Speed = GPIO_Speed_2MHz
  };
  GPIO_Init(GPIOA, &GPIO_InitStructure1);

  // Motors enable pin
  GPIO_InitTypeDef GPIO_InitStructure2 =
  {
    .GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1,  // MOTOR1_EN, MOTOR2_EN
    .GPIO_Mode  = GPIO_Mode_Out_PP,         // push pull
    .GPIO_Speed = GPIO_Speed_2MHz
  };
  GPIO_Init(GPIOC, &GPIO_InitStructure2);

  // Set output compare interrupt flags of channels configured in output
  // (CCxS=00 in TIMx_CCMRx register) when counting up and down
  TIM_CounterModeConfig(TIM2, TIM_CounterMode_CenterAligned3);

  TIM_TimeBaseInitTypeDef Timer_InitStructure =
  {
    .TIM_ClockDivision      = TIM_CKD_DIV1,
    .TIM_Prescaler          = DEFAULT_PSC,
    .TIM_Period             = PERIOD,
    .TIM_CounterMode        = TIM_CounterMode_Up
  };
  TIM_TimeBaseInit(TIM2, &Timer_InitStructure);

  // Output Compare Init :
  TIM_OCInitTypeDef OC_InitStructure;
  TIM_OCStructInit(&OC_InitStructure);
  OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;

  // Channel 1 & 2, left motor
  TIM_OC1Init(TIM2, &OC_InitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC1PolarityConfig(TIM2, TIM_OCPolarity_High); // pos pwm

  TIM_OC2Init(TIM2, &OC_InitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC2PolarityConfig(TIM2, TIM_OCPolarity_Low);  // neg pwm

  // Channel 3 & 4, right motor
  TIM_OC3Init(TIM2, &OC_InitStructure);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC3PolarityConfig(TIM2, TIM_OCPolarity_High); // pos pwm

  TIM_OC4Init(TIM2, &OC_InitStructure);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC4PolarityConfig(TIM2, TIM_OCPolarity_Low);  // neg pwm

  // Enables the TIM Capture Compare Channels
  TIM_CCxCmd(TIM2, TIM_Channel_1, TIM_CCx_Enable);
  TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable);
  TIM_CCxCmd(TIM2, TIM_Channel_3, TIM_CCx_Enable);
  TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);

  // Set default value: motors stopped
  vMotorsDisable();

  // Enables TIM peripheral Preload register on ARR
  TIM_ARRPreloadConfig(TIM2, ENABLE);

  TIM_Cmd(TIM2, ENABLE); // enable timer

  // Create the daemon
  xTaskCreate(vMotorsTask, (const signed char * const)"motorsd",
              configMINIMAL_STACK_SIZE, NULL, motorsDaemonPriority_, NULL);
}

static void vMotorsReset()
{
  previousCommand.motors = 0;
  vMotorsApplyCommands(previousCommand);
}

void vMotorsEnable()
{
  // We first stop the motors
  vMotorsReset();
  GPIO_SetBits(GPIOC, GPIO_Pin_0);
  GPIO_SetBits(GPIOC, GPIO_Pin_1);
}

void vMotorsDisable()
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_0);
  GPIO_ResetBits(GPIOC, GPIO_Pin_1);
}

static void vMotorsApplyCommands(motors_command_t cmd_)
{
  const int PWML = COMMAND_TO_PWM(cmd_.motor.left);
  const int PWMR = COMMAND_TO_PWM(cmd_.motor.right);

  // Here we disable interrupts to ensure that the same command will
  // be applied to the two motors at the same time. Moreover for a
  // single motor we want the PWMs to be fully synchronized.
  portDISABLE_INTERRUPTS();
  TIM_SetCompare1(TIM2, PWML);
  TIM_SetCompare2(TIM2, PWML);
  TIM_SetCompare3(TIM2, PWMR);
  TIM_SetCompare4(TIM2, PWMR);
  portENABLE_INTERRUPTS();
}

uint16_t uGetMotorLeftCommand()
{
  return currentCommand.motor.left;
}

uint16_t uGetMotorRightCommand()
{
  return currentCommand.motor.right;
}

void vSetMotorsCommand(int16_t left_, int16_t right_)
{
  targetCommand.motor.left = left_;
  targetCommand.motor.right = right_;
}

void vSetMotorLeftCommand(int16_t left_)
{
  targetCommand.motor.left = left_;
}

void vSetMotorRightCommand(int16_t right_)
{
  targetCommand.motor.right = right_;
}

static motors_command_t iMotorsLimitCommands(motors_command_t targ_,
                                             motors_command_t prev_)
{
  motors_command_t cmd;

  // Limit left motor command
  if (targ_.motor.left < -LIMIT_VAL || targ_.motor.left > LIMIT_VAL)
    cmd.motor.left = prev_.motor.left;
  else
    cmd.motor.left = targ_.motor.left;

  // Limit right motor command
  if (targ_.motor.right < -LIMIT_VAL || targ_.motor.right > LIMIT_VAL)
    cmd.motor.right = prev_.motor.right;
  else
    cmd.motor.right = targ_.motor.right;

  return cmd;
}

static void vMotorsTask(void* pvParameters_)
{
  portTickType time = xTaskGetTickCount();

  for (;;)
  {
    // Sample the value at this moment:
    currentCommand = iMotorsLimitCommands(targetCommand, previousCommand);
    vMotorsApplyCommands(currentCommand);

    vTaskDelayUntil(&time, 5 / portTICK_RATE_MS);
  }

}
