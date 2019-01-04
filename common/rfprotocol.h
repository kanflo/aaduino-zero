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

typedef enum {
    /** The power up message is sent when the node boots up */
    rf_powerup = 0,
    /** The hard fault message contains a small crash dump and is sent at
     *  power up
     */
    rf_hard_fault,
    /** This report is sent when the node has reestablished comms with the
     *  gateway following RF link down time. This can either be because it
     *  failed to reach the gateway or due to radio interference. The report
     *  contains one counter for each.
     */

    rf_rf_fault,
    /** Sent when the node wants to get the current time */
    rf_get_time,
    /** Gateway response to get_time */
    rf_set_time,

    /** Node temperature report */
    rf_temperature,
    /** Node battery voltage report */
    rf_battery_voltage,

    /** Sent to the node when a firmware upgrade session is about to start */
    rf_fwu_init,
    /** Sent to the gateway when the node is ready to start FWU */
    rf_fwu_start,
    /** FWU data packet */
    rf_fwu_data,
    /** FWU execution start */
    rf_fwu_execute,
    /** FWU result */
    rf_fwu_result,
} rf_frame_t;
