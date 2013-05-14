#include "libperiph/uart.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "task.h"

static xQueueHandle xUartRxQueue;
static xQueueHandle xUartTxQueue;
static xSemaphoreHandle xUartTxMutex;

void vUartInit()
{
  xUartTxMutex = xSemaphoreCreateMutex();
  xUartRxQueue = xQueueCreate(16, sizeof(char));
  xUartTxQueue = xQueueCreate(16, sizeof(char));

  // Enable interrupt UART:
  NVIC_InitTypeDef NVIC_InitStructure =
  {
    .NVIC_IRQChannel = USART1_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 7,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE,
  };
  NVIC_Init(&NVIC_InitStructure);

  // Enable clock for PC:
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  // Enable clock for UART:
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  // Enable clock for AFIO:
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);

  // Rx pin:
  GPIO_InitTypeDef GPIO_InitStruct =
    {
      .GPIO_Pin = GPIO_Pin_10,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode = GPIO_Mode_IN_FLOATING,
    };
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART_InitTypeDef UART_InitStructure;
  USART_StructInit(&UART_InitStructure);
  UART_InitStructure.USART_BaudRate = 115200,
  UART_InitStructure.USART_WordLength = USART_WordLength_8b,
  UART_InitStructure.USART_StopBits = USART_StopBits_1,
  UART_InitStructure.USART_Parity = USART_Parity_No,
  UART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
  UART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
  USART_Init(USART1, &UART_InitStructure);
  USART_Cmd(USART1, ENABLE);

  USART1->CR1 |= USART_CR1_RXNEIE;
}

char cUartGetc()
{
  char c;
  xQueueReceive(xUartRxQueue, &c, portMAX_DELAY);
  return c;
}

void vUartGets(char* s_, int size_)
{
  char c;

  for (int i = 0; i < size_; i++)
  {
    if (i == size_ - 1)
    {
      s_[i] = 0;
      break;
    }

    c = cUartGetc();

    if (c == '\r')
    {
      s_[i] = 0;
      break;
    }

    s_[i] = c;
  }
}

void vUartPutc(char c_)
{
  xQueueSend(xUartTxQueue, &c_, portMAX_DELAY);
  USART1->CR1 |= USART_CR1_TXEIE;
}

void vUartPuts(const char* s_)
{
  while (*s_)
    vUartPutc(*s_++);
}

void vUartSend(const char* s_)
{
  xSemaphoreTake(xUartTxMutex, portMAX_DELAY);
  vUartPuts(s_);
  vUartPutc('\r');
  xSemaphoreGive(xUartTxMutex);
}

void USART1_IRQHandler()
{
  static bool initializedTxPin = FALSE;
  portBASE_TYPE reschedNeeded = pdFALSE;
  char c;

  if (USART1->SR & USART_SR_RXNE) {
    c = USART1->DR;
    xQueueSendFromISR(xUartRxQueue, &c, &reschedNeeded);
  }
  else if (USART1->SR & USART_SR_TXE) {
    if (!initializedTxPin) {
      // We use registers instead of stm32 lib because we want to save
      // time inside an interrupt:
      // Tx (PA9) output push-pull 2MHz:
      GPIOA->CRH &= ~GPIO_CRH_MODE9_0;
      GPIOA->CRH |= GPIO_CRH_MODE9_1;
      GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
      GPIOA->CRH |= GPIO_CRH_CNF9_1;

      initializedTxPin = TRUE;
    }

    if (xQueueReceiveFromISR(xUartTxQueue, &c, &reschedNeeded))
      USART1->DR = c;
    else
      USART1->CR1 &= ~USART_CR1_TXEIE;
  }
  portEND_SWITCHING_ISR(reschedNeeded);
}
