#ifndef CLI_TRANSPORT_USART2_H
#define CLI_TRANSPORT_USART2_H

// cli_transport_usart2.h — USART2 / RS232 transport for TimbreOS CLI

// Call once at board init (after MX_DMA_Init + MX_USART2_UART_Init) to
// configure the DMA ring buffer and make USART2 the default CLI transport.
void usart2_transport_init(void);

// Wire into DMA1_Stream5_IRQHandler — drains the DMA ring buffer on HT/TC.
void usart2_dma_irq(void);

// Wire into USART2_IRQHandler — handles TX (TXE + TC flags only; RX is DMA).
void usart2_irq(void);

#endif // CLI_TRANSPORT_USART2_H
