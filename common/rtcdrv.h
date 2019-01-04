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

#ifndef __RTCDRV_H__
#define __RTCDRV_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief      Initialize the RTC driver
 */
void rtcdrv_init(void);

/**
 * @brief      Set and start auto wakeup period of the RTC timer
 *
 * @param[in]  period_s  The sleep period in seconds
 */
void rtcdrv_set_wakeup(uint16_t period_s);

/**
 * @brief      Get the wakeup counter
 *
 * @return     Number of wakeups of the RTC driver
 */
uint32_t rtcdrv_get_wakeup_counter(void);

/**
 * @brief      Set RTC driver time, 24 hour format (sorry Yanks ;)
 *
 * @param[in]  h          hours
 * @param[in]  m          seconds
 * @param[in]  s          seconds
 */
void rtcdrv_set_time(uint8_t h, uint8_t m, uint8_t s);

/**
 * @brief      Get current time from RTC driver
 *
 * @param      h     hour
 * @param      m     minute
 * @param      s     seconds
 * @param      pm    am (false) pm (true), may be null
 *
 */
void rtcdrv_get_time(uint8_t *h, uint8_t *m, uint8_t *s, bool *pm);

/**
 * @brief      Return the elapsed number of seconds since the RTC driver init
 *
 * @return     see above ;)
 */
uint32_t rtc_drv_get_secs(void);

#endif // __RTCDRV_H__
