#ifndef CLI_TRANSPORT_USART2_H
#define CLI_TRANSPORT_USART2_H

// cli_transport_usart2.h — USART2 / RS232 transport for TimbreOS CLI

// Call once at board init to enable RXNE interrupt and make USART2 the
// default CLI transport.
void usart2_transport_init(void);

// Call from USART2_IRQHandler — guard each call on the matching flag + IT:
//   if (LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
//       usart2_rx_irq();
//   if (LL_USART_IsActiveFlag_TXE(USART2)  && LL_USART_IsEnabledIT_TXE(USART2))
//       usart2_tx_irq();
//   if (LL_USART_IsActiveFlag_TC(USART2)   && LL_USART_IsEnabledIT_TC(USART2))
//       usart2_tc_irq();
void usart2_rx_irq(void);
void usart2_tx_irq(void);
void usart2_tc_irq(void);

#endif // CLI_TRANSPORT_USART2_H
