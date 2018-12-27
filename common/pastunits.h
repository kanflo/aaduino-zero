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

#ifndef __PASTUNITS_H__
#define __PASTUNITS_H__

/** Parameters stored in flash */
typedef enum {
    /** RFM69 key, 16 bytes */
    past_rfm_key = 1,
    /** RFM69 network id (uint32_t) */
    past_rfm_net_id,
    /** RFM69 node id (uint32_t) */
    past_rfm_node_id,
    /** RFM69 gateway id (uint32_t) */
    past_rfm_gateway_id,
    /** Max power in dBm (uint32_t) */
    past_rfm_max_power,

    /** stored as strings */
    past_boot_git_hash,
    past_app_git_hash,

    #include "fwu_pastunits.h"

    /** A past unit who's presence indicates we have a non finished upgrade and
    must not boot */
    past_upgrade_started = 0xff
} parameter_id_t;


#endif // __PASTUNITS_H__
