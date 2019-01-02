#include <gpio.h>
#include "dbg_printf.h"
#include "sw_i2c.h"
#include "hw.h"

/**
 *
 * @todo: replace this with a HW implementation ("big fat todo" that is)
 *
 */

// I2C driver for ESP8266 written for use with esp-open-rtos
// Based on https://en.wikipedia.org/wiki/I²C#Example_of_bit-banging_the_I.C2.B2C_Master_protocol

// With calling overhead, we end up at ~100kbit/s
#define CLK_HALF_PERIOD_US (1)

#define CLK_STRETCH  (10)

static void i2c_delay(void);

static bool started;

void sw_i2c_init(void)
{
    started = false;
    gpio_mode_setup(I2C1_SCL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, I2C1_SCL_PIN);
    gpio_mode_setup(I2C1_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, I2C1_SDA_PIN);
}

static void i2c_delay(void)
{
    for (uint32_t i = 0; i < 1000; i++) {   /* Wait a bit. */
        __asm__("nop");
    }
}

// Set SCL as input and return current level of line, 0 or 1
static bool read_scl(void)
{
    gpio_mode_setup(I2C1_SCL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, I2C1_SCL_PIN);
    return gpio_get(I2C1_SCL_PORT, I2C1_SCL_PIN); // Clock high, valid ACK
}

// Set SDA as input and return current level of line, 0 or 1
static bool read_sda(void)
{
    gpio_mode_setup(I2C1_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, I2C1_SDA_PIN);
    // TODO: Without this delay we get arbitration lost in sw_i2c_stop
    i2c_delay();
    return gpio_get(I2C1_SDA_PORT, I2C1_SDA_PIN); // Clock high, valid ACK
}

// Actively drive SCL signal low
static void clear_scl(void)
{
    gpio_mode_setup(I2C1_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C1_SCL_PIN);
    gpio_clear(I2C1_SCL_PORT, I2C1_SCL_PIN);
}

// Actively drive SDA signal low
static void clear_sda(void)
{
    gpio_mode_setup(I2C1_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, I2C1_SDA_PIN);
    gpio_clear(I2C1_SDA_PORT, I2C1_SDA_PIN);
}

// Output start condition
void sw_i2c_start(void)
{
    uint32_t clk_stretch = CLK_STRETCH;
    if (started) { // if started, do a restart cond
        // Set SDA to 1
        (void) read_sda();
        i2c_delay();
        while (read_scl() == 0 && clk_stretch--) ;
        // Repeated start setup time, minimum 4.7us
        i2c_delay();
    }
    if (read_sda() == 0) {
        //dbg_printf("I2C: arbitration lost in sw_i2c_start\n");
    }
    // SCL is high, set SDA from 1 to 0.
    clear_sda();
    i2c_delay();
    clear_scl();
    started = true;
}

// Output stop condition
void sw_i2c_stop(void)
{
    uint32_t clk_stretch = CLK_STRETCH;
    // Set SDA to 0
    clear_sda();
    i2c_delay();
    // Clock stretching
    while (read_scl() == 0 && clk_stretch--) ;
    // Stop bit setup time, minimum 4us
    i2c_delay();
    // SCL is high, set SDA from 0 to 1
    if (read_sda() == 0) {
        //dbg_printf("I2C: arbitration lost in sw_i2c_stop\n");
    }
    i2c_delay();
    started = false;
}

// Write a bit to I2C bus
static void i2c_write_bit(bool bit)
{
    uint32_t clk_stretch = CLK_STRETCH;
    if (bit) {
        (void) read_sda();
    } else {
        clear_sda();
    }
    i2c_delay();
    // Clock stretching
    while (read_scl() == 0 && clk_stretch--) ;
    // SCL is high, now data is valid
    // If SDA is high, check that nobody else is driving SDA
    if (bit && read_sda() == 0) {
        //dbg_printf("I2C: arbitration lost in i2c_write_bit\n");
    }
    i2c_delay();
    clear_scl();
}

// Read a bit from I2C bus
static bool i2c_read_bit(void)
{
    uint32_t clk_stretch = CLK_STRETCH;
    bool bit;
    // Let the slave drive data
    (void) read_sda();
    i2c_delay();
    // Clock stretching
    while (read_scl() == 0 && clk_stretch--) ;
    // SCL is high, now data is valid
    bit = read_sda();
    i2c_delay();
    clear_scl();
    return bit;
}

bool sw_i2c_write(uint8_t byte)
{
    bool nack;
    uint8_t bit;
    //dbg_printf("TX %02x\n", byte);
    for (bit = 0; bit < 8; bit++) {
//        dbg_printf("%d:%d\n", bit, (byte & 0x80) != 0);
        i2c_write_bit((byte & 0x80) != 0);
        byte <<= 1;
    }
    nack = i2c_read_bit();
    return !nack;
}

uint8_t sw_i2c_read(bool ack)
{
    uint8_t byte = 0;
    uint8_t bit;
    for (bit = 0; bit < 8; bit++) {
        byte = (byte << 1) | i2c_read_bit();
    }
    i2c_write_bit(ack);
    return byte;
}

bool sw_i2c_slave_write(uint8_t slave_addr, uint8_t *data, uint8_t len)
{
    bool success = false;
    do {
        sw_i2c_start();
        if (!sw_i2c_write(slave_addr << 1))
            break;
        while (len--) {
            if (!sw_i2c_write(*data++))
                break;
        }
        sw_i2c_stop();
        success = true;
    } while(0);
    return success;
}

bool sw_i2c_slave_read(uint8_t slave_addr, uint8_t data, uint8_t *buf, uint32_t len)
{
    bool success = false;
    do {
        sw_i2c_start();
        if (!sw_i2c_write(slave_addr << 1)) {
            break;
        }
        sw_i2c_write(data);
        sw_i2c_stop();
        sw_i2c_start();
        if (!sw_i2c_write(slave_addr << 1 | 1)) { // Slave address + read
            break;
        }
        while(len) {
            *buf = sw_i2c_read(len == 1);
            buf++;
            len--;
        }
        success = true;
    } while(0);
    sw_i2c_stop();
    if (!success) {
        dbg_printf("I2C: write error\n");
    }
    return success;
}

bool i2c_read_reg16(uint8_t slave_addr, uint8_t reg, uint16_t *val)
{
    uint8_t r[2];
    *val = 0;
    sw_i2c_start();
    if (!sw_i2c_write(slave_addr << 1)) {
        dbg_printf("Write 1 failed\n");
        return false;
    }
    sw_i2c_write(reg);
    sw_i2c_stop();
    sw_i2c_start();
    if (!sw_i2c_write((slave_addr << 1) | 1)) {
        dbg_printf("Write 2 failed\n");
        return false;
    }
    r[0] = sw_i2c_read(0);
    r[1] = sw_i2c_read(1);
    sw_i2c_stop();
    *val = r[0] << 8 | r[1];
    return true;
}

bool i2c_write_reg16(uint8_t slave_addr, uint8_t reg, uint16_t val)
{
    sw_i2c_start();
    if (!sw_i2c_write(slave_addr << 1)) {
        dbg_printf("Write 1 failed\n");
        return false;
    }
    sw_i2c_write(reg);
    sw_i2c_write((val >> 8) & 0xff);
    sw_i2c_write( val       & 0xff);
    sw_i2c_stop();
    return true;
}
