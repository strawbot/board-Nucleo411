// cli_transport_usart2.c — USART2 / RS232 transport for TimbreOS CLI
//
// USART2 connects to the RS232 port on the extender board and is the
// default CLI transport — active from boot, no connection handshake needed.
//
// Input routing:
//   Before each keyIn() call, EmitEvent is pointed at usart2_emit so that
//   output is automatically directed back to RS232.
//   autoEchoOff() is used — the RS232 terminal expects the device to echo.
//
// TX is interrupt-driven.  usart2_emit() (the EmitEvent target) enables the
// TXE interrupt to kick off transmission; usart2_tx_irq() drains emitq one
// byte per TXE interrupt.  When the queue empties, TXE is disabled and the TC
// interrupt is armed to detect when the last byte has fully shifted out.
//
// RX uses DMA1 Stream5 (circular mode, direct/no-FIFO) into a 256-byte ring
// buffer.  The DMA HT and TC interrupts fire at the half-way and wrap points,
// draining any new bytes into cliq and scheduling usart2_rx_action() if the
// queue was empty.  This guarantees no character is lost regardless of how
// long the cooperative action loop takes to return.
//
// Required wiring in stm32f4xx_it.c:
//   DMA1_Stream5_IRQHandler → usart2_dma_irq()
//   USART2_IRQHandler       → usart2_irq()  (TX only: TXE + TC flags)

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"

#include "tea.h"
#include "cli.h"
#include "byteq.h"
#include "printers.h"

#include "cli_transport_usart2.h"

// ── Forward declarations ──────────────────────────────────────────────────────

static void usart2_rx_action(void);

// ── CLI input queue — filled by DMA drain, consumed by usart2_rx_action ──────
static BQUEUE(100, cliq);

// ── DMA RX ring buffer ────────────────────────────────────────────────────────
//
// DMA1 Stream5 runs in circular mode: it fills dma_rx_buf[] continuously,
// wrapping at USART2_DMA_RX_SIZE.  dma_rx_head tracks the last byte we
// consumed so the drain function can copy only the new arrivals.
//
// Direct mode (FIFO disabled) is used so every byte from USART2->DR is
// written to memory immediately without waiting for a FIFO threshold.

#define USART2_DMA_RX_SIZE  256u

static uint8_t  dma_rx_buf[USART2_DMA_RX_SIZE];
static uint32_t dma_rx_head = 0;   // consumer index into dma_rx_buf[]

// Current DMA write position: where the DMA will write the *next* byte.
// NDTR counts down from USART2_DMA_RX_SIZE to 1, then reloads.
static inline uint32_t dma_rx_write_pos(void) {
    return USART2_DMA_RX_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);
}

// Copy any bytes that have arrived since the last drain into cliq and,
// if the queue was empty before, schedule usart2_rx_action().
static void usart2_dma_drain(void) {
    uint32_t wpos = dma_rx_write_pos();
    uint32_t head = dma_rx_head;

    if (head == wpos) return;   // nothing new

    bool was_empty = (qbq(cliq) == 0);

    if (wpos > head) {
        // Contiguous region — no wrap
        for (uint32_t i = head; i < wpos; i++)
            pushbq(dma_rx_buf[i], cliq);
    } else {
        // DMA has wrapped: tail of buffer, then start of buffer
        for (uint32_t i = head; i < USART2_DMA_RX_SIZE; i++)
            pushbq(dma_rx_buf[i], cliq);
        for (uint32_t i = 0; i < wpos; i++)
            pushbq(dma_rx_buf[i], cliq);
    }

    dma_rx_head = wpos;

    if (was_empty && qbq(cliq))
        later(usart2_rx_action);
}

// ── usart2_dma_irq — call from DMA1_Stream5_IRQHandler ───────────────────────
//
// Fires on HT (half-transfer, 128 bytes written) and TC (transfer complete,
// 256 bytes written / wrap to 0).  Both events just drain whatever arrived.

void usart2_dma_irq(void) {
    if (LL_DMA_IsActiveFlag_HT5(DMA1)) {
        LL_DMA_ClearFlag_HT5(DMA1);
        usart2_dma_drain();
    }
    if (LL_DMA_IsActiveFlag_TC5(DMA1)) {
        LL_DMA_ClearFlag_TC5(DMA1);
        usart2_dma_drain();
    }
}

// ── EmitEvent target — kicks off interrupt-driven TX ─────────────────────────
//
// Called cooperatively from the action queue.  If the TXE interrupt is already
// running (transmission in progress) the new bytes in emitq will be drained
// automatically — no action needed.  Otherwise enable TXE to start the ISR
// drain loop.

static void usart2_emit(void) {
    if (qbq(emitq))
        LL_USART_EnableIT_TXE(USART2);
}

// ── usart2_tx_irq — call from USART2_IRQHandler when TXE flag + IT active ────
//
// Sends one byte per interrupt.  When the queue drains: disables TXE and arms
// TC so we know when the last byte has fully shifted out of the shift register.

void usart2_tx_irq(void) {
    if (qbq(emitq)) {
        LL_USART_TransmitData8(USART2, pullbq(emitq));
    } else {
        LL_USART_DisableIT_TXE(USART2);
        LL_USART_EnableIT_TC(USART2);
    }
}

// ── usart2_tc_irq — call from USART2_IRQHandler when TC flag + IT active ─────
//
// Fires once the shift register drains after the last byte.  Disables TC to
// avoid spurious interrupts; clears the flag so the next transmission starts
// cleanly.

void usart2_tc_irq(void) {
    LL_USART_DisableIT_TC(USART2);
    LL_USART_ClearFlag_TC(USART2);
}

// ── usart2_rx_action — tea.c action, single-instance ─────────────────────────

static void usart2_rx_action(void) {
    // rx_state = USART2_RUNNING;

    // Direct output back to RS232 before processing any characters.
    when(EmitEvent, usart2_emit);
    autoEchoOff();       // RS232 terminal expects device-side echo

    while (qbq(cliq))  keyIn(pullbq(cliq));
    safe( if(qbq(cliq)) later(usart2_rx_action); )
    // safely interlock the state machine to prevent orphan bytes
}

// ── usart2_transport_init — call once from board init ────────────────────────
//
// Completes the DMA setup that CubeMX leaves to runtime (memory address,
// transfer count, interrupts) and starts the circular receive.
// Must be called after MX_DMA_Init() and MX_USART2_UART_Init().

void usart2_transport_init(void) {
    // CubeMX enables FIFO mode (threshold = 4 bytes) which would buffer
    // single keystrokes indefinitely.  Switch to direct mode so every byte
    // arriving at USART2->DR is immediately written to dma_rx_buf[].
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

    // Point the DMA at our ring buffer.
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)&USART2->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)dma_rx_buf);
    LL_DMA_SetDataLength   (DMA1, LL_DMA_STREAM_5, USART2_DMA_RX_SIZE);

    // Fire at half-full (128 bytes) and full/wrap (256 bytes) so we drain
    // promptly without needing to poll.
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_5);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);

    // Start the DMA stream and connect it to the USART.
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
    LL_USART_EnableDMAReq_RX(USART2);

    // Enable IDLE interrupt so a single byte (or short burst) gets drained
    // promptly — without this, bytes sit unprocessed until 128 accumulate.
    LL_USART_ClearFlag_IDLE(USART2);
    LL_USART_EnableIT_IDLE(USART2);

    // Establish USART2 as the default CLI transport from boot.
    when(EmitEvent, usart2_emit);
    autoEchoOff();
}

// ── usart2_irq — TX-only handler; call from USART2_IRQHandler ────────────────
//
// RX is now handled by DMA, so only TXE and TC flags matter here.

static Long pstatus = 0;

void usart2_irq(void) {
    Long status = USART2->SR;
    if ((status & USART_SR_IDLE) && LL_USART_IsEnabledIT_IDLE(USART2)) {
        LL_USART_ClearFlag_IDLE(USART2);   // cleared by SR read + DR read
        (void)USART2->DR;                  // complete the clear sequence
        usart2_dma_drain();
    }
    if ((status & USART_SR_TXE) && LL_USART_IsEnabledIT_TXE(USART2))
        usart2_tx_irq();
    if ((status & USART_SR_TC)  && LL_USART_IsEnabledIT_TC(USART2))
        usart2_tc_irq();
    pstatus |= status;
}

