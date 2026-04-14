#ifndef CLI_TRANSPORT_USART2_H
#define CLI_TRANSPORT_USART2_H

// cli_transport_usart2.h — USART2 / RS232 transport for TimbreOS CLI

// Call once at board init to enable RXNE interrupt and make USART2 the
// default CLI transport.
void usart2_transport_init(void);
void usart2_irq();

#endif // CLI_TRANSPORT_USART2_H
