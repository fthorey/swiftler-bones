#ifndef LIBPERIPH_UART_H
# define LIBPERIPH_UART_H

void uart_putc(char c);
void uart_puts(const char* s);
void uart_send(const char* s);
void uart_init();
char uart_getc();
void uart_gets(char* s, int size);

#endif
