#ifndef UART_H_
#define UART_H_

void uart_init();
bool uart_put(char ch);
int uart_get();

#endif /* UART_H_ */
