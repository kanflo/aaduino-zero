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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <rcc.h>
#include <scb.h>
#include <gpio.h>
#include <stdlib.h>
#include <flash.h>
#include <usart.h>
#include "hw.h"
#include "ringbuf.h"
#include "past.h"
#include "bootcom.h"
#include "crc16.h"
#include "spiflash.h"
#include "pastunits.h"
#include "uframe.h"
#include "protocol.h"
#include "bootcom.h"
#include "crc16.h"
#include "fwupgrade.h"
#include "libopencm3-additions.h"


/**
 * This is the AAduino Zero bootloader that eventually will take care of
 * firmware upgrades either via the UART FOTA packages stored in the external
 * flash.
 * 
 * Right now we only flash the LED a couple of times and jump to the app though.
 *
 */

#define APP_START_TIMEOUT_MS  (2500)

#ifndef MIN
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif

#define RX_BUF_SIZE       (64)
/** Something >= flash_block_size form the linker file, we need a value at compile time */
#define MAX_FRAME_SIZE   (512)

/** Flash the LED during UART comms */
#define LED_FLASH_COUNTER  (50000)

/** Linker file symbols */
extern long app_start, app_size;
extern long _bootcom_start, _bootcom_end;
extern long past_start, past_block_size, flash_block_size;

static past_t g_past;
static upgrade_reason_t reason = reason_unknown;

/** UART RX FIFO */
static ringbuf_t rx_buf;
static uint8_t buffer[2*RX_BUF_SIZE];

/** UART frame buffer */
static uint8_t frame_buffer[FRAME_OVERHEAD(MAX_FRAME_SIZE)];
static uint32_t rx_idx = 0;
static bool receiving_frame = false;

/** For keeping track of flash writing */
static uint16_t chunk_size;
static uint32_t cur_flash_address;
static uint16_t fw_crc16;

static void handle_frame(uint8_t *frame, uint32_t length);
static void send_frame(uint8_t *frame, uint32_t length);

/**
  * @brief Send ack to upgrade start and do some book keeping
  * @retval none
  */
static void send_start_response(void)
{
    DECLARE_FRAME(MAX_FRAME_LENGTH);
    PACK8(cmd_response | cmd_upgrade_start);
    PACK8(upgrade_continue);
    PACK16(chunk_size);
    PACK8(reason);
    FINISH_FRAME();
    uint32_t setting = 1;
    (void) past_write_unit(&g_past, past_upgrade_started, (void*) &setting, sizeof(setting));
    cur_flash_address = (uint32_t) &app_start;
    send_frame(_buffer, _length);
}

/**
  * @brief Handle firmware upgrade
  * @param timeout_ms timeout or 0 if no timeout
  * @retval none
  */
static void handle_upgrade(uint32_t timeout_ms)
{
    uint32_t led_counter = 0;
    bool led_on = true;
    if (fw_crc16) { /** azctl.py is expecting a response */
        send_start_response();
    }
    uint32_t start = mstimer_get();
    hw_set_led(led_on);

    while(1) {
        led_counter++;
        if (led_counter == LED_FLASH_COUNTER) {
            led_counter = 0;
            led_on = !led_on;
            hw_set_led(led_on);
        }
        uint16_t b;
        if (ringbuf_get(&rx_buf, &b)) {
            if (b == _SOF) {
                receiving_frame = true;
                rx_idx = 0;
            }
            if (receiving_frame && rx_idx < sizeof(frame_buffer)) {
                frame_buffer[rx_idx++] = b;
                if (b == _EOF) {
                    timeout_ms = 0;
                    handle_frame(frame_buffer, rx_idx);
                    receiving_frame = false;
                }
            }
        }
        if (timeout_ms) {
            if (mstimer_get() - start > timeout_ms) {
                //dbg_printf("# Timeout!\n");
                break;
            }
        }
    }
    hw_set_led(false);
}

/**
  * @brief Branch to main application
  * @retval false if app start failed
  */
static bool start_app(void)
{
    hw_deinit();
    /** Branch to the address in the reset entry of the app's exception vector */
    uint32_t *entry = (uint32_t*) (4 + (uint32_t) &app_start);
    /** Is there something there we can branch to? */
    if (((*entry) & 0xffff0000) == 0x08000000) {
        /** Initialize stack pointer of user app */
        volatile uint32_t *sp = (volatile uint32_t*) &app_start;
        __asm (
            "mov sp, %0\n" : : "r" (*sp)
        );
        ((void (*)(void))*entry)();
    }
    hw_init(&rx_buf);
    return false;
}

/**
 * @brief      Indicate system has halted
 */
static void halt(uint32_t count)
{
    while(1) {
        for (volatile int j = 0; j < 1500000; j++) ;
        for (uint32_t i = 0; i < count; i++) {
            hw_set_led(true);
            for (volatile int j = 0; j < 100000; j++) ;
            hw_set_led(false);
            for (volatile int j = 0; j < 100000; j++) ;
        }
    }
}

/**
  * @brief Send a frame on the uart
  * @param frame the frame to send
  * @param length length of frame
  * @retval None
  */
static void send_frame(uint8_t *frame, uint32_t length)
{
    do {
        usart_send_blocking(USART1, *frame);
        frame++;
    } while(--length);
}

/**
  * @brief Handle a receved frame
  * @param frame the received frame
  * @param length length of frame
  * @retval None
  */
static void handle_frame(uint8_t *frame, uint32_t length)
{
    static uint32_t counter = 0;
    command_t cmd = cmd_response;
    uint8_t *payload;
    upgrade_status_t status;
    counter++;
    int32_t payload_len = uframe_extract_payload(frame, length);
    payload = frame; // Why? Well, frame now points to the payload
    if (payload_len <= 0) {
        DECLARE_FRAME(MAX_FRAME_LENGTH);
        PACK8(cmd_response);
        PACK8(0);
        FINISH_FRAME();
        send_frame(_buffer, _length);
    } else {
        cmd = payload[0];
        switch(cmd) {
#ifdef CONFIG_BOOTLOADER_UART_FWU_SUPPORT
            case cmd_fwu_download_start:
            {
                uint16_t size, crc16;
                bool success;
                {
                    DECLARE_UNPACK(payload, length);
                    UNPACK8(cmd);
                    UNPACK16(size);
                    UNPACK16(crc16);
                }
                success = fwu_start_download(size, crc16);
                {
                    DECLARE_FRAME(MAX_FRAME_LENGTH);
                    PACK8(cmd_response | cmd_fwu_download_start);
                    PACK8(success);
                    FINISH_FRAME();
                    send_frame(_buffer, _length);
                }
                break;
            }
            case cmd_fwu_data:
            {
                fwu_got_data(&(payload[1]), payload_len - 1);
                {
                    DECLARE_FRAME(MAX_FRAME_LENGTH);
                    PACK8(cmd_response | cmd_fwu_data);
                    PACK8(1);
                    FINISH_FRAME();
                    send_frame(_buffer, _length);
                }
                break;
            }
            case cmd_fwu_upgrade:
            {
                bool success = fwu_run_upgrade();
                DECLARE_FRAME(MAX_FRAME_LENGTH);
                PACK8(cmd_response | cmd_fwu_upgrade);
                PACK8(success);
                FINISH_FRAME();
                send_frame(_buffer, _length);
                break;
            }
#endif // CONFIG_BOOTLOADER_UART_FWU_SUPPORT
            case cmd_upgrade_start:
            {
                {
                    DECLARE_UNPACK(payload, length);
                    UNPACK8(cmd);
                    UNPACK16(chunk_size);
                    chunk_size = (uint32_t) &flash_block_size;
                    UNPACK16(fw_crc16);
                }
                counter = 0;
                send_start_response();
                break;
            }
            case cmd_ping:
            {
                DECLARE_FRAME(MAX_FRAME_LENGTH);
                PACK8(cmd_response | cmd_ping);
                PACK8(1); /** success */
                FINISH_FRAME();
                send_frame(_buffer, _length);
                break;
            }
            case cmd_upgrade_data:
                if (!cur_flash_address || !fw_crc16) {
                    status = upgrade_protocol_error;
                } else if (payload_len < 0) {
                    status = upgrade_crc_error;
                } else if (cur_flash_address >= (uint32_t) &app_start + (uint32_t) &app_size) {
                    status = upgrade_overflow_error;
                } else {
                    status = upgrade_continue; /** think positive thoughts */
                    uint32_t chunk_length = payload_len - 1; /** frame type of the payload occupies 1 byte, the rest is upgrade data */
                    if (chunk_length > 0) {
                        flash_unlock();
                        flash_erase_page(cur_flash_address);
                        uint32_t word;
                        status = upgrade_continue; // Think positive
                        /** Note, payload contains 1 frame type byte and N bytes data */
                        for (uint32_t i = 0; i < (uint32_t) chunk_length; i+=4) {
                            /** @todo: Handle binaries not size aliged to 4 bytes */
                            word = payload[i+4] << 24 | payload[i+3] << 16 | payload[i+2] << 8 | payload[i+1];
                            flash_program_word(cur_flash_address + i, word);
                        }
                        cur_flash_address += chunk_length;
                        flash_lock();
                    }
                    if (chunk_length < chunk_size) { /** @todo verify code for even kb binaries */
                        uint16_t calc_crc = crc16((uint8_t*) &app_start, cur_flash_address - (uint32_t) &app_start);
                        status = fw_crc16 == calc_crc ? upgrade_success : upgrade_crc_error;
                    }
                }
                {
                    DECLARE_FRAME(MAX_FRAME_LENGTH);
                    PACK8(cmd_response | cmd_upgrade_data);
                    PACK8(status);
                    FINISH_FRAME();
                    send_frame(_buffer, _length);
                    if (status == upgrade_success) {
                        usart_wait_send_ready(USART1); /** make sure FIFO is empty */
                        (void) past_erase_unit(&g_past, past_upgrade_started);
                        cur_flash_address = 0;
                        flash_lock();
                        if (!start_app()) {
                            handle_upgrade(0); /** Try again... */
                        }
                    }
                }
                break;
            default:
            {
                DECLARE_FRAME(MAX_FRAME_LENGTH);
                PACK8(cmd_response | cmd);
                PACK8(0);
                FINISH_FRAME();
                send_frame(_buffer, _length);
                break;
            }
        }
    }
}

/**
  * @brief Ye olde main
  * @retval preferably none
  */
int main(void)
{
    bool enter_upgrade = false;
    bool has_flash;
#if 0
    void *data;
    uint32_t length;
#endif

    /** App crashed, start immediately and let it deal with it */
    if (bootcom_get_size() > 0 && bootcom_get(0) == 0xdeadc0de) {
        (void) start_app();
    }

    ringbuf_init(&rx_buf, (uint8_t*) buffer, sizeof(buffer));
    hw_init(&rx_buf);
    hw_spi_init();
    fwu_init(&g_past);
    has_flash = spiflash_probe();

    do {
        g_past.blocks[0] = (uint32_t) &past_start;
        g_past.blocks[1] = (uint32_t) &past_start + (uint32_t) &past_block_size;
        g_past._block_size = (uint32_t) &past_block_size;
        if (!past_init(&g_past)) {
            /** Not much we can do */
            enter_upgrade = true;
            reason = reason_past_failure;
            break;
        }

        if (bootcom_get_size() > 0) {
            uint32_t magic = bootcom_get(0);
            /** We got invoked by the app */
            if (magic == 0xfedebeda) {
                uint32_t temp = bootcom_get(1);
                chunk_size = temp >> 16;
                fw_crc16 = temp & 0xffff;
                enter_upgrade = true;
                reason = reason_bootcom;
                bootcom_clear();
            } else if (magic == FWU_MAGIC && has_flash) {
                (void) fwu_run_upgrade();
            }
            break;
        }

#if 0
    /** @todo: handle this, seems buggy right now */
        if (past_read_unit(&g_past, past_upgrade_started, (const void**) &data, &length)) {
            /** We have a non finished upgrade */
            enter_upgrade = true;
            reason = reason_unfinished_upgrade;
            break;
        }
#endif

#if 0
    /** Temporary LED blinking to show we're alive */
    for (int i = 0; i < 5; i++) {
        for (volatile int j = 0; j < 500000; j++) ;
        hw_set_led(true);
        for (volatile int j = 0; j < 100000; j++) ;
        hw_set_led(false);
    }
#endif

    } while(0);

    if (enter_upgrade) {
        handle_upgrade(0);
    } else {
        handle_upgrade(APP_START_TIMEOUT_MS);
        if (!start_app()) {
            reason = reason_app_start_failed;
            handle_upgrade(0); /** In case we somehow returned from the app */
        }
    }

    /** Nothing to boot */
    halt(4);
    return 0;
}
