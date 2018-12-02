/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Johan Kanflo (github.com/kanflo)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <rcc.h>
#include <gpio.h>
#include <nvic.h>
#include <spi.h>
#include <errno.h>
#include "dbg_printf.h"
#include "spi_driver.h"
#include "hw.h"

#ifdef CONFIG_SPI_IRQ_DEBUG
 #define irq_printf dbg_printf
#else
 #define irq_printf(...)
#endif


/** Used to signal xfer complete */
static volatile bool irq_done;
/** Buffers */
static volatile uint8_t *txb, *rxb;
static volatile uint32_t txb_len, rxb_len;
/** Used to mark first rx, tx */
static volatile bool first_tx = true;
static volatile bool first_rx = true;


/**
  * @brief Initialize the SPI driver
  * @retval None
  */
void spi_driver_init(void)
{
    rcc_periph_clock_enable(RCC_SPI1);
    spi_reset(SPI1);
    spi_disable(SPI1);
    spi_init_master(SPI1,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_32,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);

    spi_set_full_duplex_mode(SPI1);
    spi_disable_crc(SPI1);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    spi_enable_tx_buffer_empty_interrupt(SPI1);
    spi_enable_rx_buffer_not_empty_interrupt(SPI1);
    spi_enable_error_interrupt(SPI1);

    spi_enable(SPI1);
}


/**
  * @brief TX, and optionally RX data on the SPI bus
  * @param tx_buf transmit buffer
  * @param tx_len transmit buffer size
  * @param rx_buf receive buffer (may be NULL)
  * @param rx_len receive buffer size (may be 0)
  * @retval true if operation succeeded
  *         false if parameter or driver error
  */
bool spi_irq_transceive(uint8_t *tx_buf, uint32_t tx_len, uint8_t *rx_buf, uint32_t rx_len)
{
#ifdef CONFIG_SPI_XFER_DEBUG
    dbg_printf("\nspi_irq_transceive(tx:%d rx:%d)\n", tx_len, rx_len);
    dbg_printf("  TX:");
    for (uint32_t i = 0; i < tx_len; i++) {
        dbg_printf(" %02x", tx_buf[i]);
    }
    dbg_printf("\n");
#endif // CONFIG_SPI_XFER_DEBUG
    irq_done = false;
    txb = tx_buf;
    rxb = rx_buf;
    txb_len = tx_len;
    rxb_len = rx_len;
    first_tx = true;
    first_rx = true;

    nvic_set_priority(NVIC_SPI1_IRQ, 0);
    nvic_clear_pending_irq(NVIC_SPI1_IRQ);
    spi_enable_rx_buffer_not_empty_interrupt(SPI1);
    spi_enable_tx_buffer_empty_interrupt(SPI1);
    spi_enable_error_interrupt(SPI1);
    nvic_enable_irq(NVIC_SPI1_IRQ);

    /** @todo: decide if this belongs in here or in the module using the driver
     *   gpio_clear(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN);
     */
    spi_enable(SPI1);
    while(!irq_done) ;
    while (SPI_SR(SPI1) & SPI_SR_BSY);
    /** @todo: decide if this belongs in here or in the module using the driver
     *  gpio_set(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN);
     */

    spi_disable_rx_buffer_not_empty_interrupt(SPI1);
    spi_disable_tx_buffer_empty_interrupt(SPI1);
    spi_disable_error_interrupt(SPI1);
    nvic_disable_irq(NVIC_SPI1_IRQ);

#ifdef CONFIG_SPI_XFER_DEBUG
    dbg_printf("TX:");
    for (int i = 0; i < tx_len; i++) {
        dbg_printf(" %02x", tx_buf[i]);
    }
    dbg_printf("\n");
    dbg_printf("RX:");
    for (int i = 0; i < rx_len; i++) {
        dbg_printf(" %02x", rx_buf[i]);
    }
    dbg_printf("\n");
#endif // CONFIG_SPI_XFER_DEBUG
    return true;
}


/**
  * @brief SPI IRQ handler
  * @retval None
  */
void spi1_isr(void)
{
    uint32_t sr = SPI_SR(SPI1);
    irq_printf("\n-----\nSPI IRQ %x\n", sr);
    if (sr & SPI_SR_RXNE) {
        irq_printf("RXNE\n");
        if (rxb_len) {
            if (first_rx) {
                first_rx = false;
                *rxb = SPI_DR(SPI1);
                irq_printf("R (%02x)\n", SPI_DR(SPI1));
            } else {
                rxb_len--;
                *rxb = SPI_DR(SPI1);
                irq_printf("R %02x\n", *rxb);
                rxb++;
            }
            if (!rxb_len) {
                irq_printf("-RXNE\n");
                spi_disable_rx_buffer_not_empty_interrupt(SPI1);
            }
        } else {
//            irq_printf("R (%02x)\n", SPI_DR(SPI1));
            (void) SPI_DR(SPI1);
        }
    }
    if (sr & SPI_SR_TXE) {
        irq_printf("TXE\n");
        if (txb_len) {
            if (first_tx) {
                first_tx = false;
            } else {
                txb_len--;
            }
            if (txb_len) {
                SPI_DR(SPI1) = *txb;
                irq_printf("W %02x\n", *txb);
                txb++;
            }

            if (!txb_len) {
                irq_printf("-TXE\n");
                spi_disable_tx_buffer_empty_interrupt(SPI1);
            }
        } else {
            if (rxb_len > 0) {
                irq_printf("W (00)\n");
                SPI_DR(SPI1) = 0;
            }
        }
    }

    irq_printf("B %d:%d\n", txb_len, rxb_len);

    if (!txb_len && !rxb_len) {
        irq_done = true;
    }

    if (sr & SPI_SR_OVR) {
        (void) SPI_DR(SPI1);
        (void) SPI_SR(SPI1);
    }
}
