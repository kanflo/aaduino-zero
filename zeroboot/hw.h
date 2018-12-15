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

#ifndef __HW_H__
#define __HW_H__

#include <stdint.h>
#include <stdbool.h>
#include <gpio.h>
#include "ringbuf.h"
#include "pindefs.h"

/**
  * @brief Initialize the hardware
  * @retval None
  */
void hw_init(ringbuf_t *usart_rx_buf);

/**
 * @brief      Deinitialize certain hardware blocks before jumping to the
 *             application
 */
void hw_deinit(void);

/**
  * @brief Initialize the hardware
  * @param on Turn on if, wll, on
  * @retval None
  */
void hw_set_led(bool on);

/**
  * @brief Measure vcc
  * @retval vcc in millivolts
  */
uint16_t vcc_measure(void);

/**
  * @brief Initialize SPI1
  * @retval None
  */
void hw_spi_init(void);

#endif // __HW_H__
