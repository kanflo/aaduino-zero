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

#ifndef __TMP102_H__
#define __TMP102_H__

#include <stdint.h>
#include <stdbool.h>

/**
  * @brief Initialize the TMP102 sensor
  * @retval true if sensor was found
  */
bool tmp102_init(void);

// Returns the temperature in degrees C
float tmp102_readTempC(void);
// Converts readTempC result to degrees F
float tmp102_readTempF(void);
// Switch sensor to low power mode
void tmp102_sleep(void);
// Wakeup and start running in normal power mode
void tmp102_wakeup(void);
// Returns state of Alert register
bool tmp102_alert(void);
// Sets T_LOW (degrees C) alert threshold
void tmp102_setLowTempC(float temperature);
// Sets T_HIGH (degrees C) alert threshold
void tmp102_setHighTempC(float temperature);
// Sets T_LOW (degrees F) alert threshold
void tmp102_setLowTempF(float temperature);
// Sets T_HIGH (degrees F) alert threshold
void tmp102_setHighTempF(float temperature);
// Reads T_LOW register in C
float tmp102_readLowTempC(void);
// Reads T_HIGH register in C
float tmp102_readHighTempC(void);
// Reads T_LOW register in F
float tmp102_readLowTempF(void);
// Reads T_HIGH register in F
float tmp102_readHighTempF(void);

// Set the conversion rate (0-3)
// 0 - 0.25 Hz
// 1 - 1 Hz
// 2 - 4 Hz (default)
// 3 - 8 Hz
void tmp102_setConversionRate(uint8_t rate);

// Enable or disable extended mode
// 0 - disabled (-55C to +128C)
// 1 - enabled  (-55C to +150C)
void tmp102_setExtendedMode(bool mode);

// Set the polarity of Alert
// 0 - Active LOW
// 1 - Active HIGH
void tmp102_setAlertPolarity(bool polarity);

// Set the number of consecutive faults
// 0 - 1 fault
// 1 - 2 faults
// 2 - 4 faults
// 3 - 6 faults
void tmp102_setFault(uint8_t faultSetting);

// Set Alert type
// 0 - Comparator Mode: Active from temp > T_HIGH until temp < T_LOW
// 1 - Thermostat Mode: Active when temp > T_HIGH until any read operation occurs
void tmp102_setAlertMode(bool mode);

#endif // __TMP102_H__
