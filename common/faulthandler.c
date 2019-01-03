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

#include <stdint.h>
#include <nvic.h>
#include <scb.h>
#include "bootcom.h"
#include "rtcdrv.h"
#ifdef CONFIG_VERBOSE_FAULT_HANDLER
#include "dbg_printf.h"
#endif // CONFIG_VERBOSE_FAULT_HANDLER

/**
 * The hard fault handler save a crash dump in the bootcom area in the following
 * format (9x 32 bit words):
 *
 *   r0  r1  r2  r3  r12
 *   lr  pc  psr uptime
 *
 *  Uptime in seconds, taken from the RTC driver. A node can send this
 *  information to its gateway letting it know about the firmware health of
 *  deployed nodes.
 */

void hard_fault_handler_c(uint32_t stack[]);

/**
  * @brief Hard fault handler, save a crash dump in the bootcom area
  * @retval none
  */
void hard_fault_handler_c(uint32_t stack[])
{
#ifdef CONFIG_VERBOSE_FAULT_HANDLER
    dbg_printf("\n\n**** GURU MEDITATION ****\n");
    dbg_printf("r0    : 0x%08x\n", stack[0]);
    dbg_printf("r1    : 0x%08x\n", stack[1]);
    dbg_printf("r2    : 0x%08x\n", stack[2]);
    dbg_printf("r3    : 0x%08x\n", stack[3]);
    dbg_printf("r12   : 0x%08x\n", stack[4]);
    dbg_printf("lr    : 0x%08x\n", stack[5]);
    dbg_printf("pc    : 0x%08x\n", stack[6]);
    dbg_printf("psr   : 0x%08x\n", stack[7]);
    dbg_printf("uptime  %ds\n", rtc_drv_get_secs());
#endif // CONFIG_VERBOSE_FAULT_HANDLER
    bootcom_clear();
    bootcom_put(0xdeadc0de);
    for (uint32_t i = 0; i < 8; i++) {
        bootcom_put(stack[i]);
    }
    bootcom_put(rtc_drv_get_secs());
    scb_reset_system();
    while(1);
}


void __attribute__((naked)) hard_fault_handler(void)
{
    __asm volatile(
        "movs r0, #4\n"
        "mov  r1, lr\n"
        "tst  r0, r1\n" /* Check EXC_RETURN[2] */
        "beq 1f\n"
        "mrs r0, psp\n"
        "b 2f\n"
        "1:mrs r0,msp\n"
        "2:ldr r1,=hard_fault_handler_c\n"
        "bx r1\n"
        : /* no output */
        : /* no input */
        : "r0" /* clobber */
    );
}
