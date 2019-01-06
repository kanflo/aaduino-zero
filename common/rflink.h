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

#ifndef __RFLINK_H__
#define __RFLINK_H__

#define RFLINK_MAX_FRAME   (60)

typedef enum {
    rfl_req_ack = 1 << 0,
    rfl_ack     = 1 << 1,
    rfl_lat     = 1 << 2
} rflink_flag_t;

typedef enum {
    /** Error: tx failed due to LBT */
    txstatus_err_lbt = -100,
    /** Error: gateway did not respond */
    txstatus_err_gw,
    /** Error: got ack but sequence in ack mismatched */
    txstatus_err_ack,
    /** Success: got ack */
    txstatus_ok = 0,
    /** Success: got ack and gateway requested LAT */
    txstatus_lat
} rflink_tx_status_t;

#define FRAME_COUNTER_SHIFT   (4)
#define FRAME_COUNTER_MASK  (0xf)
#define FRAME_FLAGS_SHIFT     (0)
#define FRAME_FLAGS_MASK    (0xf)
#define FRAME_COUNTER(_cntr_flags) (((_cntr_flags) >> FRAME_COUNTER_SHIFT) & FRAME_COUNTER_MASK)
#define FRAME_FLAGS(_cntr_flags)   ((_cntr_flags) & FRAME_FLAGS_MASK)

typedef struct {
    uint8_t _dst;
    uint8_t _src;
    uint8_t _cntr_flags; // [7:4 counter] [3:0 flags]
    uint8_t payload[RFLINK_MAX_FRAME];
    int     rssi; // Valid on RX
} __attribute__((packed)) rflink_frame_t;


/**
 * @brief      Set node ID for this link node
 *
 * @param[in]  node_id  Node id
 */
void rflink_setNodeId(uint8_t node_id);

/**
 * @brief      Set network ID for this link node
 *
 * @param[in]  net_id  Network id
 */
void rflink_setNetworkId(uint8_t net_id);

/**
 * @brief      Send frame over RF link
 *
 * @param[in]  dst      Destinaton address
 * @param      payload  Payload (max RFLINK_MAX_FRAME bytes)
 * @param[in]  length   Payload length
 *
 * @return     rflink_tx_status_t
 */
rflink_tx_status_t rflink_sendFrame(uint8_t dst, rflink_frame_t *frame, uint8_t length);

/**
 * @brief      Receive a frame
 *
 * @param      src      Address of transmitter
 * @param      frame    Frame for payload
 * @param      length   Length of received payload
 *
 * @return     true if frame was received
 */
bool rflink_receiveFrame(uint8_t *src, rflink_frame_t *frame, uint8_t *length);

#endif // __RFLINK_H__
