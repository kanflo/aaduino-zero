/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Johan Kanflo (github.com/kanflo)
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

#include <nvic.h>
#include <rcc.h>
#include <pwr.h>
#include <flash.h>
#include <libopencmsis/core_cm3.h>
#include "tmp102.h"
#include "rfm69.h"
#include "spiflash.h"
#include "rtcdrv.h"
#include "lowpower.h"

/**
 * @brief      Enter stop mode, duh.
 */
static void enter_stop_mode(void)
{
    /** 6.4.1: If ULP=1 and FWU = 0: Exiting from low-power mode occurs only
     * when the VREFINT is ready */
    PWR_CR &= ~PWR_CR_FWU;
    PWR_CR |= ~PWR_CR_ULP;

    /** internal voltage regulator in low-power mode */
    PWR_CR |= PWR_CR_LPRUN;
    PWR_CR |= PWR_CR_LPSDSR;

    pwr_disable_power_voltage_detect();
    pwr_voltage_regulator_low_power_in_stop();

    SCB_SCR |= SCB_SCR_SLEEPDEEP;

    pwr_clear_wakeup_flag();

    /** Note to self. We could use pwr_set_standby_mode() if sleeping for so
     *  we can live with the "energy penalty" of a reboot. Something is wrong
     *  with entering this mode as we consume 5mA in standby mode :-/
     */
    pwr_set_stop_mode();
    __WFI();
}

/**
 * @brief      Enter stop mode and sleep for specified time.
 *             Wake up using the RTC
 *
 * @param[in]  time_s  Time to sleep in seconds.
 */
void lp_sleep(uint32_t time_s)
{
    rtcdrv_set_wakeup(time_s);

    /** Sleep peripherals */
    systick_deinit();
    rfm69_sleep();
    tmp102_sleep();
    adc_disable();
    spiflash_sleep();

    /** @todo: we will crash randomly when leaving stop mode and switch to HSI16
     *  if WS=0
     */
    flash_set_ws(1);

    /** And finally... */
    enter_stop_mode();

    /** We're running on the MSI right now */
    rcc_osc_on(RCC_HSI16);
    rcc_wait_for_osc_ready(RCC_HSI16);
    rcc_set_sysclk_source(RCC_HSI16);
    flash_set_ws(0);

    /** @todo: only wake peripherals when needed */
    systick_init();
    tmp102_wakeup();
    adc_init();
    spiflash_wakeup();
}
