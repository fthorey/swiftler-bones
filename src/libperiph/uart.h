#ifndef LIBPERIPH_UART_H
# define LIBPERIPH_UART_H

void vUartPutc(char c_);
void vUartPuts(const char* s_);
void vUartSend(const char* s_);
void vUartInit();
char cUartGetc();
void vUartGets(char* s_, int size_);

#endif
