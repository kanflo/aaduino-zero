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

#include <rtc.h>
#include <nvic.h>
#include <rcc.h>
#include <rtc.h>
#include <exti.h>
#include <pwr.h>
#include <libopencmsis/core_cm3.h>
#include "rtcdrv.h"


static uint32_t rtc_wakeup_counter = 0;
static uint32_t rtc_wakeup_period;
static uint32_t rtc_time = 0;

/** As of libopencm3 #1155c056, these are missing from rtc_common_l1f024.h  */
#ifndef RTC_PM_SHIFT
 #define RTC_PM_SHIFT (22)
#endif // RTC_PM_SHIFT

#ifndef RTC_PM_MASK
 #define RTC_PM_MASK (1)
#endif // RTC_PM_MASK


void rtc_isr(void)
{
//    rtc_handler(0, 0);
    rtc_clear_wakeup_flag();
//    PWR_CR |= PWR_CR_CWUF;
    exti_reset_request(EXTI20);
    rtc_wakeup_counter++;
    rtc_time += rtc_wakeup_period;
}

/**
 * @brief      Initialize the RTC driver
 */
void rtcdrv_init(void)
{

    /** 1. Enable the power interface clock */
    RCC_APB1ENR |= RCC_APB1ENR_PWREN;

    /** 2. Set the DBP bit in the PWR_CR register (see Section 6.4.1 of RM0376). */
    PWR_CSR |= PWR_CR_DBP;

    rtc_unlock();

    /** 3. Select the RTC clock source through RTCSEL[1:0] bits in RCC_CSR register. */
    /** RM0376 7.3.1 p. 184 */
    RCC_CR &= ~(RCC_CR_RTCPRE_MASK << RCC_CR_RTCPRE_SHIFT);
    RCC_CR |= RCC_CR_RTCPRE_DIV2 << RCC_CR_RTCPRE_SHIFT;

    /** RM0376 7.3.21 p. 215 */
    RCC_CSR &= ~(RCC_CSR_RTCSEL_MASK << RCC_CSR_RTCSEL_SHIFT);
    RCC_CSR |= RCC_CSR_RTCSEL_LSE << RCC_CSR_RTCSEL_SHIFT;

    /** 4. Enable the RTC clock by programming the RTCEN bit in the RCC_CSR register. */
    RCC_CSR |= RCC_CSR_RTCEN;

    RTC_ISR |= RTC_ISR_INIT;
    while(!(RTC_ISR & RTC_ISR_INITF)) ;
    /** @todo: document these magic numbers */
    uint32_t sync = 255;
    uint32_t async = 127;
    rtc_set_prescaler(sync, async);

    /** Set current time (to zero that is :) */
    RTC_SSR = 0;
    RTC_TR = 0;
    RTC_DR = 0;

    RTC_ISR &= ~RTC_ISR_INIT;
    rtc_lock();

    /** Block write access to RTC registers */
    PWR_CSR &= ~PWR_CR_DBP;
}

/**
 * @brief      Set and start auto wakeup period of the RTC timer
 *
 * @param[in]  period_s  The sleep period in seconds
 */
void rtcdrv_set_wakeup(uint16_t period_s)
{
    rtc_wakeup_period = period_s;
    exti_enable_request(EXTI20);
    exti_set_trigger(EXTI20, EXTI_TRIGGER_RISING);

    nvic_enable_irq(NVIC_RTC_IRQ);
    nvic_set_priority(NVIC_RTC_IRQ, 1);

    rtc_unlock();
    rtc_set_wakeup_time(period_s - 1, RTC_CR_WUCLKSEL_SPRE);
    RTC_CR |= RTC_CR_WUTIE;
    /** Without this the code works following a cold start but not a warm start */
    rtc_clear_wakeup_flag();
    rtc_lock();
}

/**
 * @brief      Get the wakeup counter
 *
 * @return     Number of wakeups of the RTC driver
 */
uint32_t rtcdrv_get_wakeup_counter(void)
{
    return rtc_wakeup_counter;
}

/**
 * @brief      Set RTC driver time, 24 hour format (sorry Yanks ;)
 *
 * @param[in]  h          hours
 * @param[in]  m          seconds
 * @param[in]  s          seconds
 */
void rtcdrv_set_time(uint8_t h, uint8_t m, uint8_t s)
{
    /** According to 26.4.7 */
    rtc_unlock();
    /** Enter init mode, stop RTC */
    RTC_ISR |= RTC_ISR_INIT;
    while(!(RTC_ISR & RTC_ISR_INITF)) ;
    /** @todo: implement rtcdrv_set_date */
    RTC_DR = 0;
    RTC_TR =
        ((h / 10) & RTC_TR_HT_MASK) << RTC_TR_HT_SHIFT |
        ((h % 10) & RTC_TR_HU_MASK) << RTC_TR_HU_SHIFT |
        ((m / 10) & RTC_TR_MNT_MASK) << RTC_TR_MNT_SHIFT |
        ((m % 10) & RTC_TR_MNU_MASK) << RTC_TR_MNU_SHIFT |
        ((s / 10) & RTC_TR_ST_MASK) << RTC_TR_ST_SHIFT |
        ((s % 10) & RTC_TR_SU_MASK) << RTC_TR_SU_SHIFT;

    /** Set 24 hour format */
    RTC_CR |= RTC_CR_FMT;
    /** Exit init mode */
    RTC_ISR &= ~RTC_ISR_INIT;
    rtc_lock();
}

/**
 * @brief      Get current time from RTC driver
 *
 * @param      h     hour
 * @param      m     minute
 * @param      s     seconds
 * @param      pm    am (false) pm (true), may be null
 *
 */
void rtcdrv_get_time(uint8_t *h, uint8_t *m, uint8_t *s, bool *pm)
{
    uint32_t tr = RTC_TR;
    uint32_t ht = (tr >> RTC_TR_HT_SHIFT) & RTC_TR_HT_MASK;
    uint32_t hu = (tr >> RTC_TR_HU_SHIFT) & RTC_TR_HU_MASK;
    uint32_t mt = (tr >> RTC_TR_MNT_SHIFT) & RTC_TR_MNT_MASK;
    uint32_t mu = (tr >> RTC_TR_MNU_SHIFT) & RTC_TR_MNU_MASK;
    uint32_t st = (tr >> RTC_TR_ST_SHIFT) & RTC_TR_ST_MASK;
    uint32_t su = (tr >> RTC_TR_SU_SHIFT) & RTC_TR_SU_MASK;
    *h = 10 * ht + hu;
    *m = 10 * mt + mu;
    *s = 10 * st + su;
    if (pm) {
        *pm = (tr >> RTC_PM_SHIFT) & RTC_PM_MASK;
    }
}

/**
 * @brief      Return the elapsed number of seconds since the RTC driver init
 *
 * @return     see above ;)
 */
uint32_t rtc_drv_get_secs(void)
{
    return rtc_time;

}
