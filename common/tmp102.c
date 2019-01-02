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

#include "dbg_printf.h"
#include "tmp102.h"
#include "sw_i2c.h"


static bool extended = false;

// TMP102 register map
#define TEMPERATURE_REG    (0x00)
#define CONFIG_REG         (0x01)
#define TEMP_LOW_REG       (0x02)
#define TEMP_HIGH_REG      (0x03)

#define TMP102_ADDRESS     (0x48)


bool tmp102_init(void)
{
    uint16_t config = 0;
    if (i2c_read_reg16(TMP102_ADDRESS, CONFIG_REG, &config) && config != 0) {
        extended = config & (1 << 4);
        dbg_printf("TMP102 found, config is 0x%04x (%s)\n", config, extended ?  "13 bit" : "12 bit");
        return true;
    } else {
        dbg_printf("No TMP102 found\n");
        return false;
    }
}

// Returns the temperature in degrees C
float tmp102_readTempC(void)
{
    int16_t temp;
    int16_t digitalTemp;

    if (!i2c_read_reg16(TMP102_ADDRESS, TEMPERATURE_REG, (uint16_t*) &temp)) {
           return 0;
    } else {
        if (extended) {
            // Combine bytes to create a signed int
            digitalTemp = ((temp>>8) << 5) | ((temp&0xff) >> 3);
            // Temperature data can be + or -, if it should be negative,
            // convert 13 bit to 16 bit and use the 2s compliment.
            if (digitalTemp > 0xFFF) {
                digitalTemp |= 0xE000;
            }
        } else {
            // Combine bytes to create a signed int
            digitalTemp = ((temp>>8) << 4) | ((temp&0xff) >> 4);
            // Temperature data can be + or -, if it should be negative,
            // convert 12 bit to 16 bit and use the 2s compliment.
            if (digitalTemp > 0x7FF) {
                digitalTemp |= 0xF000;
            }
        }
    }

    // Convert digital reading to analog temperature (1-bit is equal to 0.0625 C)
    return digitalTemp*0.0625;
}

// Converts readTempC result to degrees F
float tmp102_readTempF(void)
{
    /** @todo */
    return 0;
}

// Switch sensor to low power mode
void tmp102_sleep(void)
{
    uint16_t config = 0;
    (void) i2c_read_reg16(TMP102_ADDRESS, CONFIG_REG, &config);
    if (config) {
        /** Set SD bit for shutdown. Note to future data sheet writers: when
         *  talking about 16 bit registers, please name the bits 15..0 as
         *  "Byte 1" and "Byte 2" makes little sense to me. Thank you.
         */
        config |= 1 << 8;
        (void) i2c_write_reg16(TMP102_ADDRESS, CONFIG_REG, config);
    }
}

// Wakeup and start running in normal power mode
void tmp102_wakeup(void)
{
    uint16_t config = 0;
    (void) i2c_read_reg16(TMP102_ADDRESS, CONFIG_REG, &config);
    if (config) {
        /** Clear SD bit to wake up */
        config &= ~(1 << 8);
        (void) i2c_write_reg16(TMP102_ADDRESS, CONFIG_REG, config);
    }
}

// Returns state of Alert register
bool tmp102_alert(void)
{
    /** @todo */
    return false;
}

// Sets T_LOW (degrees C) alert threshold
void tmp102_setLowTempC(float temperature)
{
    /** @todo */
    (void) temperature;
}

// Sets T_HIGH (degrees C) alert threshold
void tmp102_setHighTempC(float temperature)
{
    /** @todo */
    (void) temperature;
}

// Sets T_LOW (degrees F) alert threshold
void tmp102_setLowTempF(float temperature)
{
    /** @todo */
    (void) temperature;
}

// Sets T_HIGH (degrees F) alert threshold
void tmp102_setHighTempF(float temperature)
{
    (void) temperature;
}

// Reads T_LOW register in C
float tmp102_readLowTempC(void)
{
    return 0;
}

// Reads T_HIGH register in C
float tmp102_readHighTempC(void)
{
    /** @todo */
    return 0;
}

// Reads T_LOW register in F
float tmp102_readLowTempF(void)
{
    /** @todo */
    return 0;
}

// Reads T_HIGH register in F
float tmp102_readHighTempF(void)
{
    /** @todo */
    return 0;
}

// Set the conversion rate (0-3)
// 0 - 0.25 Hz
// 1 - 1 Hz
// 2 - 4 Hz (default)
// 3 - 8 Hz
void tmp102_setConversionRate(uint8_t rate)
{
    /** @todo */
    (void) rate;
}

// Enable or disable extended mode
// 0 - disabled (-55C to +128C)
// 1 - enabled  (-55C to +150C)
void tmp102_setExtendedMode(bool mode)
{
    /** @todo */
    (void) mode;
}

// Set the polarity of Alert
// 0 - Active LOW
// 1 - Active HIGH
void tmp102_setAlertPolarity(bool polarity)
{
    /** @todo */
    (void) polarity;
}

// Set the number of consecutive faults
// 0 - 1 fault
// 1 - 2 faults
// 2 - 4 faults
// 3 - 6 faults
void tmp102_setFault(uint8_t faultSetting)
{
    /** @todo */
    (void) faultSetting;
}

// Set Alert type
// 0 - Comparator Mode: Active from temp > T_HIGH until temp < T_LOW
// 1 - Thermostat Mode: Active when temp > T_HIGH until any read operation occurs
void tmp102_setAlertMode(bool mode)
{
    /** @todo */
    (void) mode;
}
