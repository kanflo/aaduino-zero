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


/* This module defines the serial interface protocol.
 *
 * The basic frame payload is [<cmd>] [<optional payload>]* to which the device 
 * will respond [cmd_response | <cmd>] [success] [<response data>]*
 */

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    cmd_ping = 1,
    /** For historical reasons... */
    cmd_upgrade_start = 9,
    cmd_upgrade_data,
    cmd_fwu_download_start,
    cmd_fwu_data,
    cmd_fwu_upgrade,
    cmd_fwu_downgrade,
    cmd_response = 0x80
} command_t;

typedef enum {
    upgrade_continue = 0, /** device sent go-ahead for continued upgrade */
    upgrade_bootcom_error, /** device found errors in the bootcom data */
    upgrade_crc_error, /** crc verification of downloaded upgrade failed */
    upgrade_erase_error, /** device encountered error while erasing flash */
    upgrade_flash_error, /** device encountered error while writing to flash */
    upgrade_overflow_error, /** downloaded image would overflow flash */
    upgrade_protocol_error, /** device received upgrade data but no upgrade start */
    upgrade_success = 16 /** device received entire firmware and crc, branch verification was successful */
} upgrade_status_t;

/** The boot will report why it entered upgrade mode */
typedef enum {
    reason_unknown = 0, /** No idea why I'm here */
    reason_forced, /** User forced via button */
    reason_past_failure, /** Past init failed */
    reason_bootcom, /** App told us via bootcom */
    reason_unfinished_upgrade, /** A previous unfinished sympathy, eh upgrade */
    reason_app_start_failed /** App returned */
} upgrade_reason_t;

#define MAX_FRAME_LENGTH (2*16) // Based on the cmd_status reponse frame (fully escaped)


/*
 *    *** Command types ***
 *
 * === Pinging ===
 * The ping command is sent by the host to check if the device is online.
 *
 *  HOST:   [cmd_ping]
 *  DEVICE: [cmd_response | cmd_ping] [1]
 *
 *
 * === Upgrade sessions ===
 * When the cmd_upgrade_start packet is received, the device prepares for
 * an upgrade session:
 *  1. The upgrade packet chunk size is determined based on the host's request
 *     and is written into the bootcom RAM in addition with the 16 bit crc of
 *     the new firmware. and the upgrade magick.
 *  2. The device restarts.
 *  3. The booloader detecs the upgrade magic in the bootcom RAM.
 *  4. The booloader sets the upgrade flag in the PAST.
 *  5. The bootloader initializes the UART, sends the cmd_upgrade_start ack and
 *     prepares for download.
 *  6. The bootloader receives the upgrade packets, writes the data to flash
 *     and acks each packet.
 *  7. When the last packet has been received, the bootloader clears the upgrade
 *     flag in the PAST and boots the app.
 *  8. The host pings the app to check the new firmware started.
 *
 *  HOST:        [cmd_upgrade_start] [chunk_size:16] [crc:16]
 *  DEVICE (BL): [cmd_response | cmd_upgrade_start] [<upgrade_status_t>] [<chunk_size:16>]  [<upgrade_reason_t:8>]
 *
 * The host will send packets of the agreed chunk size with the device 
 * acknowledging each packet once crc checked and written to flash. A packet
 * smaller than the chunk size or with zero payload indicates the end of the
 * upgrade session. The device will now return the outcome of the 32 bit crc
 * check of the new firmware and continue on step 7 above.
 *
 * The upgrade data packets have the following format with the payload size
 * expected to be equal to what was aggreed upon in the cmd_upgrade_start packet.
 *
 *  HOST:      [cmd_upgrade_data] [<payload>]+
 *  DEVICE BL: [cmd_response | cmd_upgrade_data] [<upgrade_status_t>]
 *
 */

#endif // __PROTOCOL_H__
