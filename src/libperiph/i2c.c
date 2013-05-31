#include "FreeRTOS.h"

#include "misc.h"
#include "queue.h"
#include "task.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"

#include "libglobal/assert_param.h"
#include "libperiph/hardware.h"
#include "libperiph/i2c.h"

#define I2C_GPIOx   GPIOB
#define I2C_SCL_Pin GPIO_Pin_6
#define I2C_SDA_Pin GPIO_Pin_7

#define I2C_QUEUE_SIZE 10
static xQueueHandle xI2COutQueue;
static xQueueHandle xI2CInQueue;

void vI2CInit()
{
  // Create i2c queues
  xI2COutQueue = xQueueCreate(I2C_QUEUE_SIZE, sizeof (uint8_t));
  xI2CInQueue = xQueueCreate(I2C_QUEUE_SIZE, sizeof (uint8_t));

  // Configure I2C clock
  vI2CClockInit(I2C1);

  I2C_InitTypeDef I2C_InitStruct =
    {
      .I2C_ClockSpeed = 200000,
      .I2C_Mode = I2C_Mode_I2C,
      .I2C_DutyCycle = I2C_DutyCycle_2,
      .I2C_OwnAddress1 = 0x08,
      .I2C_Ack = I2C_Ack_Enable,
      .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
    };
  I2C_Init(I2C1, &I2C_InitStruct);

  // Enable interrupts
  I2C_ITConfig(I2C1, (I2C_IT_BUF | I2C_IT_EVT), ENABLE);
  NVIC_InitTypeDef NVIC_InitStruct =
    {
      .NVIC_IRQChannel                   = I2C1_EV_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 6,  // I2C MUST HAVE THE MAXIMUM
      .NVIC_IRQChannelSubPriority        = 0,  // PRIORITY AVAILABLE!
      .NVIC_IRQChannelCmd                = ENABLE,
    };
  NVIC_Init(&NVIC_InitStruct);

  // Configure SCL and SDA as alternate function open-drain outputs
  GPIO_InitTypeDef GPIO_InitStruct =
    {
      .GPIO_Pin   = I2C_SCL_Pin | I2C_SDA_Pin,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_AF_OD,
    };
  GPIO_Init(I2C_GPIOx, &GPIO_InitStruct);

  // Enable I2C
  I2C_Cmd(I2C1, ENABLE);
}

void I2C1_EV_IRQHandler()
{
  portBASE_TYPE xNeedsRescheduling = pdFALSE;

  // Address matched flag
  if (I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR)) {

    // Transmitter mode
    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_TRA)) {
      uint8_t data_out;
      if (xQueueReceiveFromISR(xI2COutQueue, &data_out, &xNeedsRescheduling))
	I2C_SendData(I2C1, data_out);
      else
        I2C_GenerateSTOP(I2C1, ENABLE);
    }

    // Receiver mode
    else {
      uint8_t data_in = I2C1->DR;
      static int test = 1;
      xQueueSendToBackFromISR(xI2CInQueue, &data_in, &xNeedsRescheduling);

      vGpioClockInit(GPIOC);
      GPIO_InitTypeDef init =
        {
          .GPIO_Pin   = GPIO_Pin_5,
          .GPIO_Speed = GPIO_Speed_2MHz,
          .GPIO_Mode  = GPIO_Mode_Out_PP,
        };
      GPIO_Init(GPIOC, &init);
      if (test) {
        GPIO_SetBits(GPIOC, GPIO_Pin_5);
        test = 0;
      }
      else {
        GPIO_ResetBits(GPIOC, GPIO_Pin_5);
        test = 1;
      }

    }

  }

  // Data register empty flag (Transmitter)
  else if (I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE)) {
      uint8_t data_out;
      if (xQueueReceiveFromISR(xI2COutQueue, &data_out, &xNeedsRescheduling))
        I2C_SendData(I2C1, data_out);
      else
        I2C_GenerateSTOP(I2C1, ENABLE);
  }

  // Data Register not empty flag (Receiver)
  else if (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE)) {
    uint8_t data_in = I2C1->DR;
    xQueueSendToBackFromISR(xI2CInQueue, &data_in, &xNeedsRescheduling);
  }

  // Stop detection flag
  else if (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF)) {
    I2C_Cmd(I2C1, ENABLE);
  }

  portEND_SWITCHING_ISR(xNeedsRescheduling);
}
