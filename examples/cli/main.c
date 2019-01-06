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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <flash.h>
#include <gpio.h>
#include <pwr.h>
#include "rtcdrv.h"
#include "cli.h"
#include "dbg_printf.h"
#include "hw.h"
#include "tick.h"
#include "tmp102.h"
#include "rfm69.h"
#include "rflink.h"
#include "spiflash.h"
#include "past.h"
#include "pastunits.h"
#include "lowpower.h"
#include "libopencm3-additions.h"

static past_t g_past;

/** Linker symbols */
extern long past_start, past_block_size;

/** RFM69 init done */
static bool rfm69_inited = false;


static void blinken_halt(uint32_t blink_count);
static void dump_mem(uint32_t address, uint32_t length);

#define RX_BUF_SIZE  (16)
static ringbuf_t rx_buf;
static uint8_t rx_buffer[2*RX_BUF_SIZE];

#define MAX_LINE_LENGTH  (80)

static void help_handler(uint32_t argc, char *argv[]);
static void halt_handler(uint32_t argc, char *argv[]);
static void past_format_handler(uint32_t argc, char *argv[]);
static void past_read_handler(uint32_t argc, char *argv[]);
static void past_write_handler(uint32_t argc, char *argv[]);
static void past_erase_handler(uint32_t argc, char *argv[]);
static void past_dump_handler(uint32_t argc, char *argv[]);
static void temperature_handler(uint32_t argc, char *argv[]);
static void temperature_alert_handler(uint32_t argc, char *argv[]);
static void rfm_handler(uint32_t argc, char *argv[]);
static void rtc_handler(uint32_t argc, char *argv[]);
static void sleep_handler(uint32_t argc, char *argv[]);
static void vcc_handler(uint32_t argc, char *argv[]);
static void spiflash_handler(uint32_t argc, char *argv[]);
static void flash_handler(uint32_t argc, char *argv[]);



#define DEFAULT_RTC_WAKEUP_S   (1)

cli_command_t commands[] = {
    {
        .cmd = "help",
        .handler = help_handler,
        .min_arg = 0, .max_arg = 0,
        .help = "Print help",
        .usage = ""
    },
    {
        .cmd = "halt",
        .handler = halt_handler,
        .min_arg = 0, .max_arg = 64,
        .help = "Halt the system",
        .usage = "<arg> ... <arg>"
    },
    {
        .cmd = "pastformat",
        .handler = past_format_handler,
        .min_arg = 0, .max_arg = 0,
        .help = "Format past",
        .usage = ""
    },
    {
        .cmd = "pastread",
        .handler = past_read_handler,
        .min_arg = 1, .max_arg = 1,
        .help = "Read unit from past",
        .usage = "<unit>"
    },
    {
        .cmd = "pastwrite",
        .handler = past_write_handler,
        .min_arg = 2, .max_arg = 2,
        .help = "Write unit to past",
        .usage = "<unit> <data>"
    },
    {
        .cmd = "pasterase",
        .handler = past_erase_handler,
        .min_arg = 1, .max_arg = 1,
        .help = "Erase unit from past",
        .usage = "<unit>"
    },
    {
        .cmd = "pastdump",
        .handler = past_dump_handler,
        .min_arg = 0, .max_arg = 1,
        .help = "Dump past",
        .usage = "[<size>]"
    },
    {
        .cmd = "temp",
        .handler = temperature_handler,
        .min_arg = 0, .max_arg = 0,
        .help = "Show TMP102 temperature",
        .usage = ""
    },
    {
        .cmd = "tempalert",
        .handler = temperature_alert_handler,
        .min_arg = 0, .max_arg = 2,
        .help = "Show TMP102 alert",
        .usage = ""
    },
    {
        .cmd = "rfm",
        .handler = rfm_handler,
        .min_arg = 0, .max_arg = 3,
        .help = "Handle RFM69",
        .usage = ""
    },
    {
        .cmd = "rtc",
        .handler = rtc_handler,
        .min_arg = 0, .max_arg = 5,
        .help = "Handle RTC",
        .usage = "[set <h> <m> <s>]"
    },
    {
        .cmd = "sleep",
        .handler = sleep_handler,
        .min_arg = 1, .max_arg = 1,
        .help = "Sleep in low power mode",
        .usage = "<seconds>"
    },
    {
        .cmd = "vcc",
        .handler = vcc_handler,
        .min_arg = 0, .max_arg = 0,
        .help = "Measure VCC",
        .usage = ""
    },
    {
        .cmd = "spiflash",
        .handler = spiflash_handler,
        .min_arg = 0, .max_arg = 3,
        .help = "Test SPI flash",
        .usage = ""
    },
    {
        .cmd = "flash",
        .handler = flash_handler,
        .min_arg = 0, .max_arg = 3,
        .help = "Test internal flash",
        .usage = ""
    },
};


static void help_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    (void) argv;
    for (uint32_t i = 0; i < sizeof(commands) / sizeof(cli_command_t); i++) {
        dbg_printf("%s    %s\n", commands[i].cmd, commands[i].help);        
    }
}

static void halt_handler(uint32_t argc, char *argv[])
{
    for (uint32_t i = 0; i < argc; i++) {
        dbg_printf("%d '%s'\n", i, argv[i]);
    }
    dbg_printf("Halted\n");
    blinken_halt(2);
}

static void past_format_handler(uint32_t argc, char *argv[])
{
    (void) argv;
    (void) argc;
    if (!past_format(&g_past)) {
        dbg_printf("Past formatting failed\n");
    }
    if (past_init(&g_past)) {
        dbg_printf("OK\n");
    } else {
        dbg_printf("ERROR\n");
    }
}

static void past_read_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    uint8_t *data;
    uint32_t length;
    uint32_t unit_id = atoi(argv[1]);
    if (past_read_unit(&g_past, unit_id, (const void**)&data, &length)) {
        dbg_printf("'%s' (%d bytes)\n", data, length);
        dump_mem((uint32_t) data, length);
    } else {
        dbg_printf("Unit %d not found\n", unit_id);
    }
}

static void past_write_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    uint32_t unit_id = atoi(argv[1]);
    if (past_write_unit(&g_past, unit_id, (void*)argv[2], strlen(argv[2]) + 1)) {
        dbg_printf("Wrote unit %d (%d bytes)\n", unit_id, strlen(argv[2]) + 1);
    } else {
        dbg_printf("Failed to write unit %d\n", unit_id);
    }
}

static void past_erase_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    uint32_t unit_id = atoi(argv[1]);
    if (past_erase_unit(&g_past, unit_id)) {
        dbg_printf("Erased unit %d\n", unit_id);
    } else {
        dbg_printf("Failed to erase unit %d\n", unit_id);
    }
}

static void past_dump_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    (void) argv;
    uint32_t dump_size = (uint32_t) &past_block_size;
    if (argc == 2) {
        dump_size = atoi(argv[1]);
    }
    dbg_printf("Past block 0:\n");
    dump_mem(g_past.blocks[0], dump_size);
    dbg_printf("\nPast block 1:\n");
    dump_mem(g_past.blocks[1], dump_size);
}

static void temperature_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    (void) argv;
    uint32_t t = 1000 * tmp102_readTempC();
    dbg_printf("%d.%d°C\n", t/1000, (t%1000)/100);
}

static void temperature_alert_handler(uint32_t argc, char *argv[])
{
    /** @todo Work in progress */
    switch (argc) {
        case 3:
            dbg_printf("low:%d high:%d\n", atoi(argv[1]), atoi(argv[2]));
            break;
        case 1:
            dbg_printf("%d\n", gpio_get(TEMP_ALERT_PORT, TEMP_ALERT_PIN));
            break;
        default:
            break;
    }
}

static void dump_rfm_settings(void)
{
    uint32_t *temp, len;
    char *temp2;
    dbg_printf("Node id    : ");
    if (past_read_unit(&g_past, past_rfm_node_id, (const void **) &temp, &len)) {
        dbg_printf("%d\n", *temp);
    } else {
        dbg_printf("NA\n");
    }
    dbg_printf("Network id : ");
    if (past_read_unit(&g_past, past_rfm_net_id, (const void **) &temp, &len)) {
        dbg_printf("%d\n", *temp);
    } else {
        dbg_printf("NA\n");
    }
    dbg_printf("Gateway id : ");
    if (past_read_unit(&g_past, past_rfm_gateway_id, (const void **) &temp, &len)) {
        dbg_printf("%d\n", *temp);
    } else {
        dbg_printf("NA\n");
    }
    dbg_printf("Max power  : ");
    if (past_read_unit(&g_past, past_rfm_max_power, (const void **) &temp, &len)) {
        dbg_printf("%d\n", *temp);
    } else {
        dbg_printf("NA\n");
    }
    dbg_printf("AES key    : ");
    if (past_read_unit(&g_past, past_rfm_key, (const void **) &temp2, &len)) {
        dbg_printf("%s\n", temp2);
    } else {
        dbg_printf("NA\n");
    }
}

static void rfm_init(void)
{
    uint32_t *node_id, *network_id, *gateway_id, *max_power, len;
    char *aesKey;
    if (!past_read_unit(&g_past, past_rfm_node_id, (const void **) &node_id, &len)) {
        dbg_printf("ERROR: RFM node id missing\n");
        return;
    }
    if (!past_read_unit(&g_past, past_rfm_net_id, (const void **) &network_id, &len)) {
        dbg_printf("ERROR: RFM network id missing\n");
        return;
    }
    if (!past_read_unit(&g_past, past_rfm_gateway_id, (const void **) &gateway_id, &len)) {
        dbg_printf("ERROR: RFM gateway id missing\n");
        return;
    }
    if (!past_read_unit(&g_past, past_rfm_max_power, (const void **) &max_power, &len)) {
        dbg_printf("ERROR: RFM max power missing\n");
        return;
    }
    if (!past_read_unit(&g_past, past_rfm_key, (const void **) &aesKey, &len)) {
        dbg_printf("ERROR: RFM AES key missing\n");
        return;
    }

    rfm69_setResetPin(RFM_RESET_PORT, RFM_RESET_PIN);
    rfm69_reset();
    if (!rfm69_init(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN, false)) {
        dbg_printf("ERROR: No RFM69CW found\n");
        return;
    } else {
        rfm69_sleep(); // init RF module and put it to sleep
        rfm69_setPowerDBm(*max_power); // // set output power, +13 dBm
        rfm69_setCSMA(true); // enable CSMA/CA algorithm
        rfm69_setAutoReadRSSI(true);
        (void) rfm69_setAESEncryption((void*) aesKey, 16);
        rflink_setNodeId(*node_id);
        rflink_setNetworkId(*network_id);
        dbg_printf("RFM69 init ok\n");
    }
}

static void rfm_tx(uint32_t dst, char *data)
{
    rflink_frame_t frame;
    dst &= 0xff;
#if 0
    dbg_printf("Sending %d bytes to %d\n", strlen(data), dst);
    dbg_printf("%s\n", data);
    dump_mem((uint32_t) data, strlen(data));
#endif
    memcpy((void*) frame.payload, (void*) data, strlen(data));
    uint8_t status = rflink_sendFrame(dst, &frame, strlen(data));
    if (!status) {
        dbg_printf("ERROR:No response\n");
    } else {
        dbg_printf("OK:%d:%d\n", status, frame.rssi);
    }
}

static void rfm_handler(uint32_t argc, char *argv[])
{
    char *cmd = argv[1];
    if (argc == 1) {
        dump_rfm_settings();
    } else if (argc == 2) {
        if (strcmp(cmd, "init") == 0) {
            rfm_init();
            rfm69_inited = true;
        }
    } else if (argc == 3) {
        if (strcmp(cmd, "id") == 0) {
            past_write_uint32(&g_past, past_rfm_node_id, atoi(argv[2]));
        } else if (strcmp(cmd, "net") == 0) {
            past_write_uint32(&g_past, past_rfm_net_id, atoi(argv[2]));
        } else if (strcmp(cmd, "gw") == 0) {
            past_write_uint32(&g_past, past_rfm_gateway_id, atoi(argv[2]));
        } else if (strcmp(cmd, "pwr") == 0) {
            past_write_uint32(&g_past, past_rfm_max_power, atoi(argv[2]));
        } else if (strcmp(cmd, "key") == 0) {
            if (strlen(argv[2]) != 16) {
                dbg_printf("ERROR: key must be 16 bytes\n");
            } else {
                past_write_cstr(&g_past, past_rfm_key, argv[2]);
            }
        } else {
            dbg_printf("ERROR: Illegal command\n");
        }
    } else if (argc == 4) {
        if (strcmp(cmd, "tx") == 0) {
            if (!rfm69_inited) {
                rfm_init();
                rfm69_inited = true;
            }
            rfm_tx(atoi(argv[2]), argv[3]);
        } else {
            dbg_printf("ERROR: Illegal command\n");
        }
    } else {
        dbg_printf("ERROR: Wrong number of parameters\n");
    }
}

static void rtc_handler(uint32_t argc, char *argv[])
{
    if (argc == 3 && strcmp(argv[1], "wakeup") == 0) {
        rtcdrv_set_wakeup(atoi(argv[1]));
    } if (argc == 5 && strcmp(argv[1], "set") == 0) {
        rtcdrv_set_time(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
    } else if (argc == 1) {
        uint8_t h, m, s;
        rtcdrv_get_time(&h, &m, &s, 0);
        /** @todo: find out why %02d does not work */
        dbg_printf("Time: %d%d:", h/10, h%10);
        dbg_printf("%d%d:", m/10, m%10);
        dbg_printf("%d%d\n", s/10, s%10);
        dbg_printf("RTC counter: %d\n", rtcdrv_get_wakeup_counter());
    } else {
        dbg_printf("Error: illegal RTC command\n");
    }
}

static void sleep_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    uint32_t time_s = atoi(argv[1]);
    dbg_printf("Entering stop mode for %ds\n", time_s);
    lp_sleep(time_s);
    dbg_printf("OK\n", time_s);
    rtcdrv_set_wakeup(DEFAULT_RTC_WAKEUP_S);
}

static void vcc_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    (void) argv;
    uint16_t vcc = vcc_measure();
    dbg_printf("%d.%02dV\n", vcc/1000, (vcc%1000)/10);
}

static void spiflash_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    (void) argv;

    static uint8_t write_buf[256];
    static uint8_t read_buf[256];

    /** Write 00 01 02 03 .. ff to page 0 */
    /** Write ff fe fd fc .. 00 to page 1 */
    for (uint32_t i = 0; i < sizeof(write_buf); i++) {
        write_buf[i] = i;
    }
    dbg_printf("Writing page 0\n");
    if (!spiflash_write(0, sizeof(write_buf), (uint8_t*) write_buf)) {
        dbg_printf("Failed\n");
        return;
    }

    for (uint32_t i = 0; i < sizeof(write_buf); i++) {
        write_buf[i] = ~i;
    }
    dbg_printf("Writing page 1\n");
    if (!spiflash_write(256, sizeof(write_buf), (uint8_t*) write_buf)) {
        dbg_printf("Failed\n");
        return;
    }

    for (uint32_t i = 0; i < sizeof(write_buf); i++) {
        write_buf[i] = i;
    }

    /** Verify writes */
    memset(read_buf, 0xcd, sizeof(read_buf));
    dbg_printf("Reading page 0\n");
    if (!spiflash_read(0, sizeof(read_buf), (uint8_t*) read_buf)) {
        dbg_printf("Failed\n");
        return;
    }
    for (uint32_t i = 0; i < sizeof(write_buf); i++) {
        if (write_buf[i] != read_buf[i]) {
            dbg_printf("Failed\n");
        }
    }

    for (uint32_t i = 0; i < sizeof(write_buf); i++) {
        write_buf[i] = ~i;
    }
    memset(read_buf, 0xcd, sizeof(read_buf));
    dbg_printf("Reading page 1\n");
    if (!spiflash_read(256, sizeof(read_buf), (uint8_t*) read_buf)) {
        dbg_printf("Failed\n");
        return;
    }
    for (uint32_t i = 0; i < sizeof(write_buf); i++) {
        if (write_buf[i] != read_buf[i]) {
            dbg_printf("Failed\n");
        }
    }

    /** Test page erase */
    dbg_printf("Erasing page 0\n");
    if (!spiflash_erase(0, sizeof(write_buf))) {
        dbg_printf("Failed\n");
        return;
    }
    if (!spiflash_read(0, sizeof(read_buf), (uint8_t*) read_buf)) {
        dbg_printf("Failed\n");
        return;
    }
    for (uint32_t i = 0; i < sizeof(read_buf); i++) {
        if (read_buf[i] != 0xff) {
            dbg_printf("Failed\n");
        }
    }

    dbg_printf("Erasing page 1\n");
    if (!spiflash_erase(256, sizeof(write_buf))) {
        dbg_printf("Failed\n");
        return;
    }
    if (!spiflash_read(256, sizeof(read_buf), (uint8_t*) read_buf)) {
        dbg_printf("Failed\n");
        return;
    }
    for (uint32_t i = 0; i < sizeof(read_buf); i++) {
        if (read_buf[i] != 0xff) {
            dbg_printf("Erase failed at %d (%02x)\n", i, read_buf[i]);
        }
    }


    /** Test chip erase, Write data to page 0, erase chip, check page 0 */
    dbg_printf("Testing chip erase\n");

    for (uint32_t i = 0; i < sizeof(write_buf); i++) {
        write_buf[i] = i;
    }
    if (!spiflash_write(0, sizeof(write_buf), (uint8_t*) write_buf)) {
        dbg_printf("Failed\n");
        return;
    }
    if (!spiflash_chip_erase()) {
        dbg_printf("Failed\n");
        return;
    }
    if (!spiflash_read(0, sizeof(read_buf), (uint8_t*) read_buf)) {
        dbg_printf("Failed\n");
        return;
    }
    for (uint32_t i = 0; i < sizeof(read_buf); i++) {
        if (read_buf[i] != 0xff) {
            dbg_printf("Erase failed at %d (%02x)\n", i, read_buf[i]);
        }
    }

    dbg_printf("Success!\n");
}

static void flash_handler(uint32_t argc, char *argv[])
{
    (void) argc;
    (void) argv;
    dbg_printf("STM32L0 Flash Test\n");
    uint32_t addr = 0x08007c00; /** Last kb */
    uint32_t page_size = 128;
    volatile uint8_t *p = (uint8_t*) addr;

    dbg_printf("Testing write\n");
    flash_unlock();
    for (uint32_t i = 0; i < page_size; i+=4) {
        flash_program_word(addr + i, 0xffffffff);
    }
    flash_lock();
    for (uint32_t i = 0; i < page_size; i++) {
        if (p[i] != 0xff) {
            dbg_printf(" Write failed at %d: 0x%02x\n", i, p[i]);
            dump_mem(addr, page_size);
            break;
        }
    }

    dbg_printf("Testing erase\n");
    flash_unlock();
    flash_erase_page(addr);
    flash_lock();
    for (uint32_t i = 0; i < page_size; i++) {
        if (p[i] != 0) {
            dbg_printf(" Erase failed at %d: 0x%02x\n", i, p[i]);
            dump_mem(addr, page_size);
            break;
        }
    }

    dbg_printf("OK\n");
}

static void blinken_halt(uint32_t blink_count)
{
    delay_ms(1);
    while(1) {
        for (uint32_t i = 0; i < blink_count; i++) {
            hw_set_led(true);
            delay_ms(100);
            hw_set_led(false);
            delay_ms(100);
        }
        delay_ms(1000);
    }
}

#define LINE_WIDTH  (16)

static void dump_mem(uint32_t address, uint32_t length)
{
    uint8_t *p = (uint8_t*) address;
    dbg_printf("%08x...%08x:", address, address + length - 1);
    for (uint32_t i = 0; i < length; i++) {
        if (i % LINE_WIDTH == 0) {
            dbg_printf("\n");
            dbg_printf("  %08x : ", address + i);
        }
        dbg_printf(" %02x", p[i]);
    }
    dbg_printf("\n");
}

/**
  * @brief Ye olde main
  * @retval preferably none
  */
int main(void)
{
    uint32_t i = 0;
    char line[MAX_LINE_LENGTH];
    line[0] = 0;

    ringbuf_init(&rx_buf, (uint8_t*) rx_buffer, sizeof(rx_buffer));
    hw_init(&rx_buf);
    /** @todo: utilize pwr_enable_power_voltage_detect(...) */

    rtcdrv_init();
    rtcdrv_set_wakeup(DEFAULT_RTC_WAKEUP_S);

    g_past.blocks[0] = (uint32_t) &past_start;
    g_past.blocks[1] = (uint32_t) &past_start + (uint32_t) &past_block_size;
    g_past._block_size = (uint32_t) &past_block_size;

    if (!past_init(&g_past)) {
        dbg_printf("Error: past init failed!\n");
        dbg_printf("Past block 0:\n");
        dump_mem(g_past.blocks[0], 64);
        dbg_printf("Past block 1:\n");
        dump_mem(g_past.blocks[1], 64);
        blinken_halt(3);
    }

    dbg_printf("\n\nWelcome to the AAduino Zero CLI\n");

    if (!spiflash_probe()) {
        dbg_printf("No SPI flash found\n");
    } else {
        dbg_printf("Found SPI flash %s\n", spiflash_get_desc());
    }

    if (tmp102_init()) {
        uint32_t t = 1000 * tmp102_readTempC();
        dbg_printf("Temperature is %d.%d°C\n", t/1000, (t%1000)/100);
    }

    uint16_t vcc = vcc_measure();
    dbg_printf("VCC is %d.%02dV\n", vcc/1000, (vcc%1000)/10);

    dbg_printf("Try 'help <return>' for, well, help.\n");
    dbg_printf("%% ");
    while(1) {
        uint16_t b;
        /** @todo. replace UART ringbuffer with event buffer like in OpenDPS */
        while (ringbuf_get(&rx_buf, &b)) {
            if (b == '\r') {
                // Don't care
            } else if (b == '\n') {
                dbg_printf("\n");
                if (i > 0) {
                    line[i] = 0;
                    cli_run(commands, sizeof(commands) / sizeof(cli_command_t), line);
                    i = 0;
                    line[0] = 0;
                }
                dbg_printf("%% ");
            } else if (i < MAX_LINE_LENGTH-2) {
                dbg_printf("%c", b & 0xff);
                line[i++] = b;
            }
        }
    }
    return 0;
}
