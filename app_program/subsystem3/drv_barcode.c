/** @file drv_barcode.c
 *
 * @brief Barcode IR module AO on GP26 (ADC0). The ADC converts every
 * 0.5 ms into its FIFO; DMA channel 11 copies each result into a 256
 * sample ring that wraps by itself, so no interrupt or re-arming is
 * needed. The Wi-Fi driver claims the lowest free DMA channels at run
 * time, so channel 11 stays clear of it.
 *
 * The kernel powers the ADC up at boot, and makes GP26 an analog input
 * (DEVCONF_ADC_PIN_INIT_0). Nothing opens the kernel's "adca" device.
 */

#include "drv_barcode.h"
#include "../app.h"

#define DRV_BARCODE_ADC_CS   0x4004C000u
#define DRV_BARCODE_ADC_FIFO 0x4004C00Cu
#define DRV_BARCODE_ADC_FCS  0x4004C008u
#define DRV_BARCODE_ADC_DIV  0x4004C010u
#define DRV_BARCODE_CS_RUN   0x9u          /* EN | START_MANY, channel 0. */
#define DRV_BARCODE_FCS_DMA  0x01000009u   /* EN | DREQ_EN, THRESH 1. */
#define DRV_BARCODE_DIV_2KHZ (23999u << 8) /* 48 MHz / 24000. */

#define DRV_BARCODE_DMA         (0x50000000u + (11u * 0x40u)) /* Ch 11. */
#define DRV_BARCODE_DMA_READ    (DRV_BARCODE_DMA + 0x0u)
#define DRV_BARCODE_DMA_WRITE   (DRV_BARCODE_DMA + 0x4u)
#define DRV_BARCODE_DMA_COUNT   (DRV_BARCODE_DMA + 0x8u)
#define DRV_BARCODE_DMA_CTRL    (DRV_BARCODE_DMA + 0xCu)
#define DRV_BARCODE_DMA_FOREVER 0xFFFFFFFFu /* Transfers: 24 days. */

/* CTRL: EN, 16-bit, write increments, 512-byte ring on the write side,
 * chained to itself (no chain), paced by DREQ_ADC (36). */
#define DRV_BARCODE_DMA_RUN                                                    \
    ((1u << 0) | (1u << 2) | (1u << 5) | (9u << 6) | (1u << 10) |              \
     (11u << 11) | (36u << 15))

/* The ring must be aligned to its size for the DMA to wrap it. */
static volatile uint16_t g_ring[DRV_BARCODE_RING] __attribute__((aligned(512)));
static uint8_t g_read; /* Next ring index to hand out; wraps at 256. */

/*!
 * @brief Start the ADC converting and the DMA filling the ring.
 */
void drv_barcode_init(void)
{
    out_w(DRV_BARCODE_ADC_DIV, DRV_BARCODE_DIV_2KHZ);
    out_w(DRV_BARCODE_ADC_FCS, DRV_BARCODE_FCS_DMA);
    out_w(DRV_BARCODE_DMA_READ, DRV_BARCODE_ADC_FIFO);
    out_w(DRV_BARCODE_DMA_WRITE, (uint32_t)g_ring); /* 32-bit addresses. */
    out_w(DRV_BARCODE_DMA_COUNT, DRV_BARCODE_DMA_FOREVER);
    out_w(DRV_BARCODE_DMA_CTRL, DRV_BARCODE_DMA_RUN);
    out_w(DRV_BARCODE_ADC_CS, DRV_BARCODE_CS_RUN);
}

/*!
 * @brief Copy out the samples written since the last call. Call at least
 * every 128 ms, or the oldest samples are overwritten.
 *
 * @param[out] p_samples Room for DRV_BARCODE_RING samples, 0 to 4095.
 * @return Number of samples copied.
 */
uint32_t drv_barcode_read(uint16_t * p_samples)
{
    /* (uint32_t): RP2040 addresses are 32 bits. */
    uint32_t const write =
        ((in_w(DRV_BARCODE_DMA_WRITE) - (uint32_t)g_ring) / 2u) %
        DRV_BARCODE_RING;
    uint32_t count = 0u;

    while (g_read != write)
    {
        p_samples[count] = g_ring[g_read];
        g_read++; /* DRV_BARCODE_RING is 256, fixed by the DMA ring. */
        count++;
    }
    return count;
}

/*** end of file ***/
