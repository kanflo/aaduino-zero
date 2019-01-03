/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Johan Kanflo (github.com/kanflo)
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
#include <usart.h>
#include <i2c.h>
#include <spi.h>
#include <flash.h>
#include <libopencmsis/core_cm3.h>
#include "dbg_printf.h"
#include "tick.h"
#ifndef CONFIG_SKIP_I2C
 #include "sw_i2c.h"
#endif // CONFIG_SKIP_I2C
#include "hw.h"
#include "spi_driver.h"

#ifdef CONFIG_RAM_VECTORS
#include <scb.h>
/** Linker file symbols */
extern uint32_t *_ram_vect_start;
extern uint32_t *_ram_vect_end;
extern uint32_t *vector_table;
static void copy_vectors(void);
#endif // CONFIG_RAM_VECTORS

static void tim2_init(void);
static void clock_init(void);
static void usart_init(bool enable_rx);
static void gpio_init(void);
static void exti_init(void);
#ifndef CONFIG_SKIP_I2C
static void i2c_init(void);
#endif // CONFIG_SKIP_I2C
static void spi_init(void);
static void lse_init(void);
//static void button_irq_init(void);

static ringbuf_t *rx_buf;


/** As of libopencm3 #1155c056, these are missing from pwr_common_v1.h */

/** FWU: Fast wakeup */
#ifndef PWR_CR_FWU
 #define PWR_CR_FWU          (1 << 10)
#endif // PWR_CR_FWU

/** ULP: Ultra-low-power mode */
#ifndef PWR_CR_ULP
 #define PWR_CR_ULP          (1 << 9)
#endif // PWR_CR_ULP


/**
  * @param usart_rx_buf pointer to UART ring buffer, may be null.
  * @retval None
  */
void hw_init(ringbuf_t *usart_rx_buf)
{
    rx_buf = usart_rx_buf;
#ifdef CONFIG_RAM_VECTORS
    copy_vectors();
#endif // CONFIG_RAM_VECTORS
    clock_init();
    systick_init();
    gpio_init();
    usart_init(usart_rx_buf != 0);
    exti_init();
    tim2_init();
#ifndef CONFIG_SKIP_I2C
    i2c_init();
#endif // CONFIG_SKIP_I2C
    spi_init();
    adc_init();
    lse_init();
}

/**
  * @brief Relocate the vector table to the internal SRAM
  * @retval None
  */
#ifdef CONFIG_RAM_VECTORS
static void copy_vectors(void)
{
    uint32_t v_size = (uint32_t) &_ram_vect_end - (uint32_t) &_ram_vect_start;
    volatile uint32_t *v_rom = (uint32_t*) &vector_table;
    volatile uint32_t *v_ram = (uint32_t*) &_ram_vect_start;
    for(uint32_t i = 0; i < v_size/4; i++) {
        v_ram[i] = v_rom[i];
    }
    SCB_VTOR = (uint32_t) &_ram_vect_start;
}
#endif // CONFIG_RAM_VECTORS

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
  * @brief Disable ADC
  * @retval None
  */
void adc_disable(void)
{
    adc_disable_vrefint();
    adc_power_off(ADC1);
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
#if 0
    /* Turn on the appropriate source for the PLL */
    if (clock->pll_source == RCC_CFGR_PLLSRC_HSE_CLK) {
        rcc_osc_on(RCC_HSE);
        rcc_wait_for_osc_ready(RCC_HSE);
    } else {
        rcc_osc_on(RCC_HSI16);
        rcc_wait_for_osc_ready(RCC_HSI16);
    }

    rcc_set_hpre(clock->hpre);
    rcc_set_ppre1(clock->ppre1);
    rcc_set_ppre2(clock->ppre2);

    rcc_periph_clock_enable(RCC_PWR);
    pwr_set_vos_scale(clock->voltage_scale);

    rcc_osc_off(RCC_PLL);
    while (rcc_is_osc_ready(RCC_PLL));

    flash_prefetch_enable();
    flash_set_ws(clock->flash_waitstates);

    /* Set up the PLL */
    rcc_set_pll_multiplier(clock->pll_mul);
    rcc_set_pll_divider(clock->pll_div);
    rcc_set_pll_source(clock->pll_source);

    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    rcc_set_sysclk_source(RCC_PLL);

    /* Set the peripheral clock frequencies used. */
    rcc_ahb_frequency = clock->ahb_frequency;
    rcc_apb1_frequency = clock->apb1_frequency;
    rcc_apb2_frequency = clock->apb2_frequency;
#else
    rcc_osc_on(RCC_HSI16);
    rcc_wait_for_osc_ready(RCC_HSI16);


    rcc_osc_off(RCC_PLL);
    while (rcc_is_osc_ready(RCC_PLL));
    flash_prefetch_enable();
    flash_set_ws(1);

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
#endif
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
//    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_SPI1);
}

#ifndef CONFIG_SKIP_I2C
/**
  * @brief Initialize I2C1
  * @retval None
  */
static void i2c_init(void)
{
#if 1
    sw_i2c_init();
#else
    /** @todo: use HW i2c */
    i2c_reset(I2C1);

    /* Setup GPIO pins for I2C1 */
    gpio_mode_setup(I2C1_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C1_SCL_PIN);
    gpio_mode_setup(I2C1_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C1_SDA_PIN);

    /* Setup I2C1 SCL, SDA pins as alternate function. */
    gpio_set_af(I2C1_SCL_PORT, I2C1_SCL_AF, I2C1_SCL_PIN);
    gpio_set_af(I2C1_SDA_PORT, I2C1_SDA_AF, I2C1_SDA_PIN);

    i2c_peripheral_disable(I2C1);

    /** @todo: clean this mess up... */
    /* APB1 is running at 16MHz. */
//    i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_16MHZ);

    /* 400KHz - I2C Fast Mode */
//    i2c_set_fast_mode(I2C1);

    /*
     * fclock for I2C is 16MHz APB2 -> cycle time 28ns, low time at 400kHz
     * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
     * Datasheet suggests 0x1e.
     */
//    i2c_set_ccr(I2C1, 0x1e);

    /*
     * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
     * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
     * Incremented by 1 -> 11.
     */
//    i2c_set_trise(I2C1, 0x0b);

    /*
     * This is our slave address - needed only if we want to receive from
     * other masters.
     */
//    i2c_set_own_7bit_slave_address(I2C1, 0x32);

    /* If everything is configured -> enable the peripheral. */


    //configure ANFOFF DNF[3:0] in CR1
//    i2c_enable_analog_filter(I2C1);
//    i2c_set_digital_filter(I2C1, I2C_CR1_DNF_DISABLED);
    //Configure PRESC[3:0] SDADEL[3:0] SCLDEL[3:0] SCLH[7:0] SCLL[7:0]
    // in TIMINGR
//    i2c_100khz_i2cclk8mhz(I2C1);
    //configure No-Stretch CR1 (only relevant in slave mode)
    i2c_enable_stretching(I2C1);
    //addressing mode
    i2c_set_7bit_addr_mode(I2C1);


    i2c_peripheral_enable(I2C1);
#endif
}
#endif // CONFIG_SKIP_I2C

/**
  * @brief Initialize SPI1
  * @retval None
  */
static void spi_init(void)
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
  * @brief Initialize USART1
  * @param enable_rx if true, enable USART RX
  * @retval None
  */
static void usart_init(bool enable_rx)
{
    /* Setup USART1 parameters. */
    rcc_periph_clock_enable(RCC_USART1);
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
    if (enable_rx) {
        usart_set_mode(USART1, USART_MODE_TX_RX);
        nvic_enable_irq(NVIC_USART1_IRQ);
        usart_enable_rx_interrupt(USART1);
    }
    dbg_printf("\n---\n");
}

void usart1_isr(void)
{
    /** @todo: this code hangs if pasting lots and lots of characters into the terminal */
    if ((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0 &&
        (USART_ISR(USART1) & USART_ISR_RXNE) != 0) {
        uint16_t ch = usart_recv(USART1);
        if ((USART_ISR(USART1) & USART_ISR_ORE) == 0 &&
            (USART_ISR(USART1) & USART_ISR_FE) == 0 &&
            (USART_ISR(USART1) & USART_ISR_PE) == 0) {
            if (rx_buf && !ringbuf_put(rx_buf, ch)) {
                //printf("ASSERT:usart1_isr:%d\n", __LINE__);
            }
        } else {
            //printf("ASSERT:usart1_isr:%d\n", __LINE__);
        }
    }
}

/**
  * @brief Initialize GPIO
  * @retval None
  */
static void gpio_init(void)
{
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_mode_setup(AUX_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, AUX_PIN);
    gpio_mode_setup(TEMP_ALERT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TEMP_ALERT_PIN);

    /** Setup GPIO pins for USART1 */
    gpio_mode_setup(USART1_RXI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART1_RXI_PIN);
    gpio_set_af(USART1_RXI_PORT, USART1_RXI_AF, USART1_RXI_PIN);
    gpio_mode_setup(USART1_TXO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART1_TXO_PIN);
    gpio_set_af(USART1_TXO_PORT, USART1_TXO_AF, USART1_TXO_PIN);

    /** RFM stuff... */
    gpio_mode_setup(RFM_RESET_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RFM_RESET_PIN);
    gpio_clear(RFM_RESET_PORT, RFM_RESET_PIN);
    gpio_mode_setup(RFM_IRQ_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, RFM_IRQ_PIN);
}

/**
  * @brief Enable EXTI where most buttons are connected
  * @retval None
  */
static void exti_init(void)
{
//    AFIO_EXTICR2 = 0x11;
//    AFIO_EXTICR3 = 0x11;
}

/**
  * @brief Set up TIM2 for injected ADC1 sampling
  * @retval None
  */
static void tim2_init(void)
{
    uint32_t timer = TIM2;
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_reset_pulse(RST_TIM2);
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(timer, 0xFF);
    timer_set_prescaler(timer, 0x8);
    timer_set_clock_division(timer, 0x0);
    timer_set_master_mode(timer, TIM_CR2_MMS_UPDATE); // Generate TRGO on every update.
    timer_enable_counter(timer);
}

#if 0
/**
  * @brief Select button ISR
  * @note BUTTON_SEL_isr & friends defined in hw.h
  * @retval None
  */
void BUTTON_SEL_isr(void)
{
    static bool falling = true;
    exti_reset_request(BUTTON_SEL_EXTI);
    if (falling) {
        longpress_begin(event_button_sel);
        exti_set_trigger(BUTTON_SEL_EXTI, EXTI_TRIGGER_RISING);
    } else {
        if (!longpress_end()) {
            // Not a long press, send short press
            event_put(event_button_sel, press_short);
        }
        exti_set_trigger(BUTTON_SEL_EXTI, EXTI_TRIGGER_FALLING);
    }
    falling = !falling;
}


/**
  * @brief Initialize IRQs for the buttons
  * @retval None
  */
static void button_irq_init(void)
{
    nvic_enable_irq(BUTTON_SEL_NVIC);
    exti_select_source(BUTTON_SEL_EXTI, BUTTON_SEL_PORT);
    exti_set_trigger(BUTTON_SEL_EXTI, EXTI_TRIGGER_FALLING);
    exti_enable_request(BUTTON_SEL_EXTI);
}
#endif

/**
  * @brief Initialize the ADC for vref int messurements
  * @retval None
  */
void adc_init(void)
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

/**
 * @brief      Enable LSE
 */
static void rcc_css_lse_enable(void)
{
    RCC_CSR |= RCC_CSR_CSSLSEON;
}

/**
  * @brief Initialize the low speed xtal
  * @retval None
  */
static void lse_init(void)
{
    /*
        The LSEON, LSEBYP, RTCSEL,LSEDRV and RTCEN bits in the RCC control and
        status register (RCC_CSR) are in the RTC domain. As these bits are write
        protected after reset, the DBP bit in the Power control register
        (PWR_CR) has to be set to be able to modify them. Refer to
        Section 6.1.2: RTC and RTC backup registers for further information.

        These bits are only reset after a RTC domain reset (see Section 6.1.2).
        Any internal or external reset does not have any effect on them.
    */

    // Allow change or LSE drive strength
    RCC_APB1ENR |= RCC_APB1ENR_PWREN;
    PWR_CR |= PWR_CR_DBP;

    RCC_CSR &= ~(RCC_CSR_LSEDRV_MASK << RCC_CSR_LSEDRV_SHIFT);
    RCC_CSR |= RCC_CSR_LSEDRV_MLOW << RCC_CSR_LSEDRV_SHIFT;

    rcc_osc_on(RCC_LSE);
    rcc_css_lse_enable();

    uint32_t start = mstimer_get();
    while (!rcc_is_osc_ready(RCC_LSE) && mstimer_get() - start < 1000 ) ;
    if (rcc_is_osc_ready(RCC_LSE)) {
        dbg_printf("LSE ready (drive strength %d)\n", (RCC_CSR >> RCC_CSR_LSEDRV_SHIFT) & RCC_CSR_LSEDRV_MASK);
    } else {
        dbg_printf("LSE failed to start! (drive strength %d)\n", (RCC_CSR >> RCC_CSR_LSEDRV_SHIFT) & RCC_CSR_LSEDRV_MASK);
    }
}
