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

#include <string.h>
#include <timer.h>
#include <rcc.h>
#include <pwr.h>
#include <adc.h>
#include <nvic.h>
#include <exti.h>
#include <spi.h>
#include <flash.h>
#include "hw.h"
#include "spi_driver.h"

static void clock_init(void);
static void adc_init(void);

/**
  * @param usart_rx_buf pointer to UART ring buffer, may be null.
  * @retval None
  */
void hw_init(void)
{
    clock_init();
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

/**
  * @brief Initialize the hardware
  * @param on LED on or off
  * @retval None
  */
void hw_set_led(bool on)
{
    on ? gpio_set(LED_PORT, LED_PIN) : gpio_clear(LED_PORT, LED_PIN);
}

/**
  * @brief Measure vcc
  * @retval vcc in millivolts
  */
uint16_t vcc_measure(void)
{
    uint16_t raw;
    adc_start_conversion_regular(ADC1);
    while (! adc_eoc(ADC1));
    raw = adc_read_regular(ADC1);
    return 1200 * 4096 / raw;  // ADC sample to millivolts
}

/**
  * @brief Initialize SPI1
  * @retval None
  */
void hw_spi_init(void)
{
    /** Setup SPI1 pins */
    gpio_mode_setup(SPI1_MOSI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI1_MOSI_PIN);
    gpio_set_af(SPI1_MOSI_PORT, SPI1_MOSI_AF, SPI1_MOSI_PIN);

    gpio_mode_setup(SPI1_MISO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI1_MISO_PIN);
    gpio_set_af(SPI1_MISO_PORT, SPI1_MISO_AF, SPI1_MISO_PIN);

    gpio_mode_setup(SPI1_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI1_SCK_PIN);
    gpio_clear(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN);
    gpio_set_af(SPI1_SCK_PORT, SPI1_SCK_AF, SPI1_SCK_PIN);

    gpio_mode_setup(SPI1_RFM_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI1_RFM_CS_PIN);
    gpio_set(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN);

    gpio_mode_setup(SPI1_FLASH_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI1_FLASH_CS_PIN);
    gpio_set(SPI1_FLASH_CS_PORT, SPI1_FLASH_CS_PIN);

    spi_driver_init();
}

/**
  * @brief Enable clocks
  * @retval None
  */
static void clock_init(void)
{
    /** Jump up to 16mhz using HSI16 as we don't have an HSE
      *  Currently no support for HSI16 in rcc_clock_setup_pll(...)
      *  @todo: enable PLL
      */
    rcc_osc_on(RCC_HSI16);
    rcc_wait_for_osc_ready(RCC_HSI16);


    rcc_osc_off(RCC_PLL);
    while (rcc_is_osc_ready(RCC_PLL));
    flash_prefetch_enable();
    flash_set_ws(0);

    /** @todo: needs to be done somewhere around here or ADC init hangs */
    adc_init();
    (void) vcc_measure();
    /** @todo: Set clock frequency according to VCC */

    /** @todo: Add mode for 4MHz HSI16 mode */

    /* Set up the PLL */
    rcc_set_pll_multiplier(RCC_CFGR_PLLMUL_MUL4);
    rcc_set_pll_divider(RCC_CFGR_PLLDIV_DIV4);
    rcc_set_pll_source(0 /** RCC_CFGR_PLLSRC_HSI_CLK */);

    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    rcc_set_sysclk_source(RCC_PLL);

    rcc_set_sysclk_source(RCC_HSI16);
    rcc_ahb_frequency = 16000000;
    rcc_apb1_frequency = 16000000;
    rcc_apb2_frequency = 16000000;

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
}

/**
  * @brief Initialize the ADC for vref int messurements
  * @retval None
  */
static void adc_init(void)
{
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    adc_power_off(ADC1);
    rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
    rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
    adc_set_single_conversion_mode(ADC1);
    adc_power_on(ADC1);
    adc_enable_vrefint();
    ADC_CHSELR(ADC1) = 1 << ADC_CHANNEL_VREF;
}
