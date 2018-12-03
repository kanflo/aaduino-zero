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

#include <stdio.h>
#include <nvic.h>
#include <rcc.h>
#include <rtc.h>
#include <exti.h>
#include <pwr.h>
#include <libopencmsis/core_cm3.h>
#include "rtcdrv.h"


static uint32_t rtc_counter = 0;


void rtc_isr(void)
{
//    rtc_handler(0, 0);
    rtc_clear_wakeup_flag();
//    PWR_CR |= PWR_CR_CWUF;
    exti_reset_request(EXTI20);
    rtc_counter++;
}

/**
  * @brief Initialize the ring buffer
  * @param ring pointer to ring buffer
  * @param buf buffer to store ring buffer data in
  * @param size size of buffer
  * @retval none
  */
void rtcdrv_init(void)
{

    /** . Enable the power interface clock by setting the PWREN bits in the RCC_APB1ENR register. */
    RCC_APB1ENR |= RCC_APB1ENR_PWREN;

    /** 2. Set the DBP bit in the PWR_CR register (see Section 6.4.1). */
    PWR_CSR |= PWR_CR_DBP;

    rtc_unlock();

    /** 3. Select the RTC clock source through RTCSEL[1:0] bits in RCC_CSR register. */
    /** RM0376 7.3.1 p. 184 */
    RCC_CR &= ~(RCC_CR_RTCPRE_MASK << RCC_CR_RTCPRE_SHIFT);
    RCC_CR |= RCC_CR_RTCPRE_DIV2 << RCC_CR_RTCPRE_SHIFT;

    /** RM0376  7.3.21 p. 215 */
    RCC_CSR &= ~(RCC_CSR_RTCSEL_MASK << RCC_CSR_RTCSEL_SHIFT);
    RCC_CSR |= RCC_CSR_RTCSEL_LSE << RCC_CSR_RTCSEL_SHIFT;

    /**4 . Enable the RTC clock by programming the RTCEN bit in the RCC_CSR register. */
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

void rtcdrv_set_wakeup(uint16_t period_s)
{
    exti_enable_request(EXTI20);
    exti_set_trigger(EXTI20, EXTI_TRIGGER_RISING);

    nvic_enable_irq(NVIC_RTC_IRQ);
    nvic_set_priority(NVIC_RTC_IRQ, 1);

    rtc_unlock();
#if 1
    rtc_set_wakeup_time(period_s - 1, RTC_CR_WUCLKSEL_SPRE);
    RTC_CR |= RTC_CR_WUTIE;
    /** Without this the code works following a cold start but not a warm start :-/ */
    rtc_clear_wakeup_flag();
#else
    RTC_CR &= ~RTC_CR_WUTE;
    while(!(RTC_ISR & RTC_ISR_WUTWF)) ;

    RTC_ISR |= RTC_ISR_WUTF /*WUTWF*/; //has to be set before writing to WUTR (manual 22.7.6)

#if 1
    /** Enable wakeup every 60s */
    RTC_WUTR = 1;
//    RTC_WUTR = 59;
    RTC_CR = RTC_CR_WUTIE | RTC_CR_WUTE | RTC_CR_WUCLKSEL_SPRE;
#else
    /** 0xffff is 32 seconds @ RTC_DIV16 */
//    RTC_WUTR = 0xffff;
//    RTC_CR = RTC_CR_WUTIE | RTC_CR_WUTE | RTC_CR_WUCLKSEL_RTC_DIV16; // enable wakeup function and interrupt, RTC/2 clock
#endif

    RTC_ISR &= ~RTC_ISR_INIT; //start the RTC (clear INIT bit)

    /** Without this the code works following a cold start but not a warm start :-/ */
    rtc_clear_wakeup_flag();
#endif
    rtc_lock();
}

uint32_t rtcdrv_get_counter(void)
{
    return rtc_counter;
}

