/**
 * @file rfm69.h
 *
 * @brief RFM69 and RFM69HW library for sending and receiving packets in connection with a STM32 controller.
 * @date January, February 2015
 * @author André Heßling
 *
 * This is a protocol agnostic driver library for handling HopeRF's RFM69 433/868/915 MHz RF modules.
 * Support is also available for the +20 dBm high power modules called RFM69HW/RFM69HCW.
 *
 * A CSMA/CA (carrier sense multiple access) algorithm can be enabled to avoid collisions.
 * If you want to enable CSMA, you should initialize the random number generator before.
 *
 * This library is written for the STM32 family of controllers, but can easily be ported to other devices.
 *
 * You have to provide your own functions for delay_ms and mstimer_get.
 * Use the SysTick timer (for example) with a 1 ms resolution which is present on all ARM controllers.
 *
 * If you want to port this library to other devices, you have to provide an SPI instance
 * derived from the SPIBase class.
 */

#ifndef RFM69_HPP_
#define RFM69_HPP_

/** @addtogroup RFM69
 * @{
 */
#define RFM69_MAX_PAYLOAD		64 ///< Maximum bytes payload

#include "stdint.h"
#include "stdbool.h"

/**
 * Valid RFM69 operation modes.
 */
typedef enum
{
  RFM69_MODE_SLEEP = 0,//!< Sleep mode (lowest power consumption)
  RFM69_MODE_STANDBY,  //!< Standby mode
  RFM69_MODE_FS,       //!< Frequency synthesizer enabled
  RFM69_MODE_TX,       //!< TX mode (carrier active)
  RFM69_MODE_RX        //!< RX mode
} RFM69Mode;

/**
 * Valid RFM69 data modes.
 */
typedef enum
{
  RFM69_DATA_MODE_PACKET = 0,                 //!< Packet engine active
  RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC = 2,   //!< Continuous mode with clock recovery
  RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC = 3,//!< Continuous mode without clock recovery
} RFM69DataMode;

bool rfm69_init(uint32_t csPort, uint32_t csPin, bool highPowerDevice);

//set this node's address
void rfm69_setAddress(uint8_t addr);

//set this node's network id
void rfm69_setNetwork(uint8_t networkID);

/**
 * Set the hardware reset pin of the RFM69 module that is connected to the controller.
 *
 * @param resetGPIO GPIO of /NRES signal (ie. GPIOA, GPIOB, ...)
 * @param resetPin Pin of /NRES signal (eg. GPIO_Pin_1)
 */
void rfm69_setResetPin(uint32_t resetPort, uint32_t resetPin);

/**
 * Set the data pin (DIO2) of the RFM69 module that is connected to the controller.
 *
 * @param dataGPIO GPIO of DIO2 signal (ie. GPIOA, GPIOB, ...)
 * @param dataPin Pin of DIO2 signal (eg. GPIO_Pin_1)
 */
void rfm69_setDataPin(uint32_t dataPort, uint32_t dataPin);

void rfm69_reset(void);

void rfm69_setFrequency(unsigned int frequency);

void rfm69_setFrequencyDeviation(unsigned int frequency);

void rfm69_setBitrate(unsigned int bitrate);

RFM69Mode rfm69_setMode(RFM69Mode mode);

void rfm69_setPowerLevel(uint8_t power);

int rfm69_setPowerDBm(int8_t dBm);

void rfm69_setHighPowerSettings(bool enable);

void rfm69_setCustomConfig(const uint8_t config[][2], unsigned int length);

int rfm69_send(uint8_t *data, uint8_t dataLength);

int rfm69_receive(char* data, unsigned int dataLength);

void rfm69_sleep(void);

/**
 * Gets the last "cached" RSSI reading.
 *
 * @note This only gets the latest reading that was requested by readRSSI().
 *
 * @return RSSI value in dBm.
 */
int rfm69_getRSSI(void);

void rfm69_setOOKMode(bool enable);

void rfm69_setDataMode(RFM69DataMode dataMode);

/**
 * Enable/disable the automatic reading of the RSSI value during packet reception.
 *
 * Default is off (no reading).
 *
 * @param enable true or false
 */
void rfm69_setAutoReadRSSI(bool enable);

/**
 * Enable/disable the CSMA/CA (carrier sense) algorithm before sending a packet.
 *
 * @param enable true or false
 */
void rfm69_setCSMA(bool enable);

void rfm69_continuousBit(bool bit);

void rfm69_dumpRegisters(void);

void rfm69_setPASettings(uint8_t forcePA);

bool rfm69_setAESEncryption(uint8_t* aesKey, unsigned int keyLength);

uint8_t rfm69_readRegister(uint8_t reg);
void rfm69_writeRegister(uint8_t reg, uint8_t value);



#endif /* RFM69_HPP_ */

/** @}
 *
 */
