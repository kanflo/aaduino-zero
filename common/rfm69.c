/**
 * @file rfm69.c
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
 * You have to provide your own functions for delay_ms() and mstimer_get().
 * Use the SysTick timer (for example) with a 1 ms resolution which is present on all ARM controllers.
 *
 * If you want to port this library to other devices, you have to provide an SPI instance
 * derived from the SPIBase class.
 */

/** @addtogroup RFM69
 * @{
 */

#include <stdlib.h>
#include <string.h>
#include <spi.h>
#include <gpio.h>
#include "dbg_printf.h"
#include "hw.h"
#include "tick.h"
#include "rfm69.h"
#include "rfm69_register.h"
#include "spi_driver.h"

// available frequency bands
#define RF69_315MHZ           31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ           43
#define RF69_868MHZ           86
#define RF69_915MHZ           91

#define TIMEOUT_MODE_READY    100 ///< Maximum amount of time until mode switch [ms]
#define TIMEOUT_PACKET_SENT   100 ///< Maximum amount of time until packet must be sent [ms]
#define TIMEOUT_CSMA_READY    500 ///< Maximum CSMA wait time for channel free detection [ms]
#define CSMA_RSSI_THRESHOLD   -85 ///< If RSSI value is smaller than this, consider channel as free [dBm]

static uint8_t _address;
static uint32_t _csPort, _csPin;
static uint32_t _resetPort, _resetPin;
static uint32_t _dataPort, _dataPin;
static bool _init;
static RFM69Mode _mode;
static bool _highPowerDevice;
static uint8_t _powerLevel;
static int _rssi;
static bool _autoReadRSSI;
static bool _ookEnabled;
static RFM69DataMode _dataMode;
static bool _highPowerSettings;
static bool _csmaEnabled;
static char _rxBuffer[RFM69_MAX_PAYLOAD];
static unsigned int _rxBufferLength;

static inline void rfm69_chipSelect(void);
static inline void rfm69_chipUnselect(void);
static void rfm69_clearFIFO(void);
static void rfm69_waitForModeReady(void);
static bool rfm69_waitForPacketSent(void);
static int rfm69_readRSSI(void);
static bool rfm69_channelFree(void);
static int rfm69_receive_internal(char* data, uint8_t dataLength);


/** RFM69 base configuration after init().
 *
 * Change these to your needs or call rfm69_setCustomConfig() after module init.
 */
#define RFM69_FREQ   (RF69_868MHZ)
#define RFM69_NET_ID (100)

static const uint8_t rfm69_base_config[][2] =
{
    // LowPowerLab/RFM69 compatible

    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

    /* 0x07 */ { REG_FRFMSB, (uint8_t) (RFM69_FREQ==RF69_315MHZ ? RF_FRFMSB_315 : (RFM69_FREQ==RF69_433MHZ ? RF_FRFMSB_433 : (RFM69_FREQ==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
    /* 0x08 */ { REG_FRFMID, (uint8_t) (RFM69_FREQ==RF69_315MHZ ? RF_FRFMID_315 : (RFM69_FREQ==RF69_433MHZ ? RF_FRFMID_433 : (RFM69_FREQ==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
    /* 0x09 */ { REG_FRFLSB, (uint8_t) (RFM69_FREQ==RF69_315MHZ ? RF_FRFLSB_315 : (RFM69_FREQ==RF69_433MHZ ? RF_FRFLSB_433 : (RFM69_FREQ==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
    /* 0x30 */ { REG_SYNCVALUE2, RFM69_NET_ID }, // NETWORK ID
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}

#if 0
#if 1 // Moteino compatible
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, //no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, //default:4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, //default:5khz, (FDEV + BitRate/2 <= 500Khz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

    /* 0x07 */ { REG_FRFMSB, (RFM69_FREQ==RF69_315MHZ ? RF_FRFMSB_315 : (RFM69_FREQ==RF69_433MHZ ? RF_FRFMSB_433 : (RFM69_FREQ==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
    /* 0x08 */ { REG_FRFMID, (RFM69_FREQ==RF69_315MHZ ? RF_FRFMID_315 : (RFM69_FREQ==RF69_433MHZ ? RF_FRFMID_433 : (RFM69_FREQ==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
    /* 0x09 */ { REG_FRFLSB, (RFM69_FREQ==RF69_315MHZ ? RF_FRFLSB_315 : (RFM69_FREQ==RF69_433MHZ ? RF_FRFLSB_433 : (RFM69_FREQ==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout=-18+OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout=-14+OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout=-11+OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, //over current protection (default is 95mA)

    ///* 0x18*/ { REG_LNA,  RF_LNA_ZIN_200 | RF_LNA_CURRENTGAIN }, //as suggested by mav here: http://lowpowerlab.com/forum/index.php/topic,296.msg1571.html

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4khz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, //(BitRate < 2 * RxBw)
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, //DIO0 is the only IRQ we're using
    /* 0x29 */ { REG_RSSITHRESH, 220 }, //must be set to dBm = (-Sensitivity / 2) - default is 0xE4=228 so -114dBm
    ///* 0x2d */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2e */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2f */ { REG_SYNCVALUE1, 0x2D },      //attempt to make this compatible with sync1 byte of RFM12B lib
    /* 0x30 */ { REG_SYNCVALUE2, RFM69_NET_ID }, //NETWORK ID
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, //in variable length mode: the max frame size, not used in TX
    //* 0x39 */ { REG_NODEADRS, nodeID }, //turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, //TX on FIFO not empty
    /* 0x3d */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
    {255, 0}
#else
    {0x01, 0x04}, // RegOpMode: Standby Mode
    {0x02, 0x00}, // RegDataModul: Packet mode, FSK, no shaping
    {0x03, 0x0C}, // RegBitrateMsb: 10 kbps
    {0x04, 0x80}, // RegBitrateLsb
    {0x05, 0x01}, // RegFdevMsb: 20 kHz
    {0x06, 0x48}, // RegFdevLsb
    {0x07, 0xD9}, // RegFrfMsb: 868,15 MHz
    {0x08, 0x09}, // RegFrfMid
    {0x09, 0x9A}, // RegFrfLsb
    {0x18, 0x88}, // RegLNA: 200 Ohm impedance, gain set by AGC loop
    {0x19, 0x4C}, // RegRxBw: 25 kHz
    {0x2C, 0x00}, // RegPreambleMsb: 3 bytes preamble
    {0x2D, 0x03}, // RegPreambleLsb
    {0x2E, 0x88}, // RegSyncConfig: Enable sync word, 2 bytes sync word
    {0x2F, 0x41}, // RegSyncValue1: 0x4148
    {0x30, 0x48}, // RegSyncValue2
    {0x37, 0xD0}, // RegPacketConfig1: Variable length, CRC on, whitening
    {0x38, 0x40}, // RegPayloadLength: 64 bytes max payload
    {0x3C, 0x8F}, // RegFifoThresh: TxStart on FifoNotEmpty, 15 bytes FifoLevel
    {0x58, 0x1B}, // RegTestLna: Normal sensitivity mode
    {0x6F, 0x30}, // RegTestDagc: Improved margin, use if AfcLowBetaOn=0 (default)
#endif
#endif
};

// Clock constants. DO NOT CHANGE THESE!
#define RFM69_XO               32000000    ///< Internal clock frequency [Hz]
#define RFM69_FSTEP            61.03515625 ///< Step width of synthesizer [Hz]

/**
 * Reset the RFM69 module using the external reset line.
 *
 * @note Use rfm69_setResetPin() before calling this function.
 */
void rfm69_reset()
{
  if (!_resetPort)
    return;

  _init = false;

  // generate reset impulse
  gpio_set(_resetPort, _resetPin);
  delay_ms(1);
  gpio_clear(_resetPort, _resetPin);

  // wait until module is ready
  delay_ms(10);

  _mode = RFM69_MODE_STANDBY;
}

/**
 * Initialize the RFM69 module.
 * A base configuration is set and the module is put in standby mode.
 *
 * @return Always true
 */
/**
 * RFM69 default constructor. Use init() to start working with the RFM69 module.
 *
 * @param spi Pointer to a SPI device
 * @param csGPIO GPIO of /CS line (ie. GPIOA, GPIOB, ...)
 * @param csPin Pin of /CS line (eg. GPIO_Pin_1)
 * @param highPowerDevice Set to true, if this is a RFM69Hxx device (default: false)
 */
bool rfm69_init(uint32_t csPort, uint32_t csPin, bool highPowerDevice)
{
  _address = 0;
  _csPort = csPort;
  _csPin = csPin;
  _resetPort = _resetPin = 0;
  _init = false;
  _mode = RFM69_MODE_STANDBY;
  _highPowerDevice = highPowerDevice;
  _powerLevel = 0;
  _rssi = -127;
  _ookEnabled = false;
  _autoReadRSSI = false;
  _dataMode = RFM69_DATA_MODE_PACKET;
  _dataPort = _dataPin = 0;
  _highPowerSettings = false;
  _csmaEnabled = false;
  _rxBufferLength = 0;

  gpio_mode_setup(_csPin, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, _csPin);
  gpio_set(_csPin, _csPort);

  uint8_t tmp = rfm69_readRegister(0x10);
  if (tmp != 0x24) {
    dbg_printf("Error: revision register is not 0x24 but 0x%02x\n", tmp);
  }

  rfm69_writeRegister(0x25, 0x01); // DIO0 signals PayloadReady in RX, TxReady in TX
  tmp = rfm69_readRegister(0x25);

  // set base configuration
  rfm69_setCustomConfig(rfm69_base_config, sizeof(rfm69_base_config) / 2);

  // set PA and OCP settings according to RF module (normal/high power)
  rfm69_setPASettings(0);

  // clear FIFO and flags
  rfm69_clearFIFO();

  _init = true;

  return _init;
}

//set this node's address
void rfm69_setAddress(uint8_t addr)
{
  _address = addr;
  rfm69_writeRegister(REG_NODEADRS, _address);
}

//set this node's network id
void rfm69_setNetwork(uint8_t networkID)
{
  rfm69_writeRegister(REG_SYNCVALUE2, networkID);
}

/**
 * Set the carrier frequency in Hz.
 * After calling this function, the module is in standby mode.
 *
 * @param frequency Carrier frequency in Hz
 */
void rfm69_setFrequency(unsigned int frequency)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode)
    rfm69_setMode(RFM69_MODE_STANDBY);

  // calculate register value
  frequency /= RFM69_FSTEP;

  // set new frequency
  rfm69_writeRegister(0x07, frequency >> 16);
  rfm69_writeRegister(0x08, frequency >> 8);
  rfm69_writeRegister(0x09, frequency);
}

/**
 * Set the FSK frequency deviation in Hz.
 * After calling this function, the module is in standby mode.
 *
 * @param frequency Frequency deviation in Hz
 */
void rfm69_setFrequencyDeviation(unsigned int frequency)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode)
    rfm69_setMode(RFM69_MODE_STANDBY);

  // calculate register value
  frequency /= RFM69_FSTEP;

  // set new frequency
  rfm69_writeRegister(0x05, frequency >> 8);
  rfm69_writeRegister(0x06, frequency);
}

/**
 * Set the bitrate in bits per second.
 * After calling this function, the module is in standby mode.
 *
 * @param bitrate Bitrate in bits per second
 */
void rfm69_setBitrate(unsigned int bitrate)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode)
    rfm69_setMode(RFM69_MODE_STANDBY);

  // calculate register value
  bitrate = RFM69_XO / bitrate;

  // set new bitrate
  rfm69_writeRegister(0x03, bitrate >> 8);
  rfm69_writeRegister(0x04, bitrate);
}

/**
 * Read a RFM69 register value.
 *
 * @param reg The register to be read
 * @return The value of the register
 */
uint8_t rfm69_readRegister(uint8_t reg)
{
  // sanity check
  if (reg > 0x7f)
    return 0;

  // read value from register
  rfm69_chipSelect();

  uint8_t tx[] = { reg };
  uint8_t rx[1], value = 0;
  if (spi_irq_transceive(tx, sizeof(tx), rx, sizeof(rx))) {
    value = rx[0];
  }
  rfm69_chipUnselect();

  return value;
}

/**
 * Write a RFM69 register value.
 *
 * @param reg The register to be written
 * @param value The value of the register to be set
 */
void rfm69_writeRegister(uint8_t reg, uint8_t value)
{
  // sanity check
  if (reg > 0x7f)
    return;

  // transfer value to register and set the write flag
  rfm69_chipSelect();

  uint8_t tx[] = { reg | 0x80, value };
  if (spi_irq_transceive(tx, sizeof(tx), 0, 0)) {
  }

  rfm69_chipUnselect();
}

/**
 * Acquire the chip.
 */
void rfm69_chipSelect()
{
  gpio_clear(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN);
}

/**
 * Switch the mode of the RFM69 module.
 * Using this function you can manually select the RFM69 mode (sleep for example).
 *
 * This function also takes care of the special registers that need to be set when
 * the RFM69 module is a high power device (RFM69Hxx).
 *
 * This function is usually not needed because the library handles mode changes automatically.
 *
 * @param mode RFM69_MODE_SLEEP, RFM69_MODE_STANDBY, RFM69_MODE_FS, RFM69_MODE_TX, RFM69_MODE_RX
 * @return The new mode
 */
RFM69Mode rfm69_setMode(RFM69Mode mode)
{
  if ((mode == _mode) || (mode > RFM69_MODE_RX))
    return _mode;

  // set new mode
  rfm69_writeRegister(0x01, mode << 2);

  // set special registers if this is a high power device (RFM69HW)
  if (true == _highPowerDevice)
  {
    switch (mode)
    {
    case RFM69_MODE_RX:
      // normal RX mode
      if (true == _highPowerSettings)
        rfm69_setHighPowerSettings(false);
      break;

    case RFM69_MODE_TX:
      // +20dBm operation on PA_BOOST
      if (true == _highPowerSettings)
        rfm69_setHighPowerSettings(true);
      break;

    default:
      break;
    }
  }

  _mode = mode;

  return _mode;
}

/**
 * Release the chip.
 */
void rfm69_chipUnselect()
{
  gpio_set(SPI1_RFM_CS_PORT, SPI1_RFM_CS_PIN);
}

/**
 * Enable/disable the power amplifier(s) of the RFM69 module.
 *
 * PA0 for regular devices is enabled and PA1 is used for high power devices (default).
 *
 * @note Use this function if you want to manually override the PA settings.
 * @note PA0 can only be used with regular devices (not the high power ones!)
 * @note PA1 and PA2 can only be used with high power devices (not the regular ones!)
 *
 * @param forcePA If this is 0, default values are used. Otherwise, PA settings are forced.
 *                0x01 for PA0, 0x02 for PA1, 0x04 for PA2, 0x08 for +20 dBm high power settings.
 */
void rfm69_setPASettings(uint8_t forcePA)
{
  // disable OCP for high power devices, enable otherwise
  rfm69_writeRegister(0x13, 0x0A | (_highPowerDevice ? 0x00 : 0x10));

  if (0 == forcePA)
  {
    if (true == _highPowerDevice)
    {
      // enable PA1 only
      rfm69_writeRegister(0x11, (rfm69_readRegister(0x11) & 0x1F) | 0x40);
    }
    else
    {
      // enable PA0 only
      rfm69_writeRegister(0x11, (rfm69_readRegister(0x11) & 0x1F) | 0x80);
    }
  }
  else
  {
    // PA settings forced
    uint8_t pa = 0;

    if (forcePA & 0x01)
      pa |= 0x80;

    if (forcePA & 0x02)
      pa |= 0x40;

    if (forcePA & 0x04)
      pa |= 0x20;

    // check if high power settings are forced
    _highPowerSettings = (forcePA & 0x08) ? true : false;
    rfm69_setHighPowerSettings(_highPowerSettings);

    rfm69_writeRegister(0x11, (rfm69_readRegister(0x11) & 0x1F) | pa);
  }
}

/**
 * Set the output power level of the RFM69 module.
 *
 * @param power Power level from 0 to 31.
 */
void rfm69_setPowerLevel(uint8_t power)
{
  if (power > 31)
    power = 31;

  rfm69_writeRegister(0x11, (rfm69_readRegister(0x11) & 0xE0) | power);

  _powerLevel = power;
}

/**
 * Enable the +20 dBm high power settings of RFM69Hxx modules.
 *
 * @note Enabling only works with high power devices.
 *
 * @param enable true or false
 */
void rfm69_setHighPowerSettings(bool enable)
{
  // enabling only works if this is a high power device
  if (true == enable && false == _highPowerDevice)
    enable = false;

  rfm69_writeRegister(0x5A, enable ? 0x5D : 0x55);
  rfm69_writeRegister(0x5C, enable ? 0x7C : 0x70);
}

/**
 * Reconfigure the RFM69 module by writing multiple registers at once.
 *
 * @param config Array of register/value tuples
 * @param length Number of elements in config array
 */
void rfm69_setCustomConfig(const uint8_t config[][2], unsigned int length)
{
  for (unsigned int i = 0; i < length; i++)
  {
    rfm69_writeRegister(config[i][0], config[i][1]);
  }
}

/**
 * Send a packet over the air.
 *
 * After sending the packet, the module goes to standby mode.
 * CSMA/CA is used before sending if enabled by function setCSMA() (default: off).
 *
 * @note A maximum amount of RFM69_MAX_PAYLOAD bytes can be sent.
 * @note This function blocks until packet has been sent.
 *
 * @param data Pointer to buffer with data
 * @param dataLength Size of buffer
 *
 * @return Number of bytes that have been sent, zero if packet could not be sent
 */
int rfm69_send(uint8_t *data, uint8_t dataLength)
{
  // switch to standby and wait for mode ready, if not in sleep mode
  if (RFM69_MODE_SLEEP != _mode)
  {
    rfm69_setMode(RFM69_MODE_STANDBY);
    rfm69_waitForModeReady();
  }

  // clear FIFO to remove old data and clear flags
  rfm69_clearFIFO();

  // limit max payload
  if (dataLength > RFM69_MAX_PAYLOAD)
    dataLength = RFM69_MAX_PAYLOAD;

  // payload must be available
  if (0 == dataLength)
    return 0;

  /* Wait for a free channel, if CSMA/CA algorithm is enabled.
   * This takes around 1,4 ms to finish if channel is free */
  if (true == _csmaEnabled)
  {
    // Restart RX
    rfm69_writeRegister(0x3D, (rfm69_readRegister(0x3D) & 0xFB) | 0x20);

    // switch to RX mode
    rfm69_setMode(RFM69_MODE_RX);

    // wait until RSSI sampling is done; otherwise, 0xFF (-127 dBm) is read

    // RSSI sampling phase takes ~960 µs after switch from standby to RX
    uint32_t timeEntry = mstimer_get();
    while (((rfm69_readRegister(0x23) & 0x02) == 0) && ((mstimer_get() - timeEntry) < 10));

    while ((false == rfm69_channelFree()) && ((mstimer_get() - timeEntry) < TIMEOUT_CSMA_READY))
    {
      // wait for a random time before checking again
      delay_ms(rand() % 10);

      /* try to receive packets while waiting for a free channel
       * and put them into a temporary buffer */
      int bytesRead;
      if ((bytesRead = rfm69_receive_internal(_rxBuffer, RFM69_MAX_PAYLOAD)) > 0)
      {
        _rxBufferLength = bytesRead;

        // module is in RX mode again

        // Restart RX and wait until RSSI sampling is done
        rfm69_writeRegister(0x3D, (rfm69_readRegister(0x3D) & 0xFB) | 0x20);
        timeEntry = mstimer_get();
        while (((rfm69_readRegister(0x23) & 0x02) == 0) && ((mstimer_get() - timeEntry) < 10));
      }
    }

    rfm69_setMode(RFM69_MODE_STANDBY);
  }

  uint8_t tx[dataLength + 2];
  tx[0] = 0x00 | 0x80;
  tx[1] = dataLength;
  for (uint32_t i = 0; i < dataLength; i++) {
    tx[i+2] = data[i];
  }
  // transfer packet to FIFO
  rfm69_chipSelect();
  if (!spi_irq_transceive(tx, sizeof(tx), 0, 0)) {
    dataLength = 0;
  }
  rfm69_chipUnselect();

  // start radio transmission
  rfm69_setMode(RFM69_MODE_TX);

  // wait for packet sent
  if (!rfm69_waitForPacketSent()) {
    dataLength = 0;
  }

  // go to standby
  rfm69_setMode(RFM69_MODE_STANDBY);

  return dataLength;
}

/**
 * Clear FIFO and flags of RFM69 module.
 */
static void rfm69_clearFIFO()
{
  // clear flags and FIFO
  rfm69_writeRegister(0x28, 0x10);
}

/**
 * Wait until the requested mode is available or timeout.
 */
static void rfm69_waitForModeReady()
{
  uint32_t timeEntry = mstimer_get();

  while (((rfm69_readRegister(0x27) & 0x80) == 0) && ((mstimer_get() - timeEntry) < TIMEOUT_MODE_READY));
}

/**
 * Put the RFM69 module to sleep (lowest power consumption).
 */
void rfm69_sleep()
{
  rfm69_setMode(RFM69_MODE_SLEEP);
}

/**
 * Put the RFM69 module in RX mode and try to receive a packet.
 *
 * @note The module resides in RX mode.
 *
 * @param data Pointer to a receiving buffer
 * @param dataLength Maximum size of buffer
 * @return Number of received bytes; 0 if no payload is available.
 */
int rfm69_receive(char* data, unsigned int dataLength)
{
  // check if there is a packet in the internal buffer and copy it
  if (_rxBufferLength > 0)
  {
    // copy only until dataLength, even if packet in local buffer is actually larger
    memcpy(data, _rxBuffer, dataLength);

    unsigned int bytesRead = _rxBufferLength;

    // empty local buffer
    _rxBufferLength = 0;
    return bytesRead;
  }
  else
  {
    // regular receive
    return rfm69_receive_internal(data, dataLength);
  }
}

/**
 * Put the RFM69 module in RX mode and try to receive a packet.
 *
 * @note This is an internal function.
 * @note The module resides in RX mode.
 *
 * @param data Pointer to a receiving buffer
 * @param dataLength Maximum size of buffer
 * @return Number of received bytes; 0 if no payload is available.
 */
static int rfm69_receive_internal(char* data, uint8_t dataLength)
{
  uint8_t i = 0, frameLength = 0;
  // go to RX mode if not already in this mode
  if (RFM69_MODE_RX != _mode)
  {
    rfm69_setMode(RFM69_MODE_RX);
    rfm69_waitForModeReady();
  }

  // check for flag PayloadReady
  if (rfm69_readRegister(0x28) & 0x04)
  {
    // go to standby before reading data
    rfm69_setMode(RFM69_MODE_STANDBY);

    // get FIFO content
    uint8_t bytesRead = 0;

    // read until FIFO is empty or buffer length exceeded
    while ((rfm69_readRegister(0x28) & 0x40) && (bytesRead < dataLength))
    {
      if (bytesRead == 0) {
        /** First byte from the FIFO is the length, don't return it as 'payload' */
        frameLength = rfm69_readRegister(0x00);
      } else {
        data[i++] = rfm69_readRegister(0x00);
      }
      bytesRead++;
    }

    // automatically read RSSI if requested
    if (true == _autoReadRSSI)
    {
      rfm69_readRSSI();
    }

    // go back to RX mode
    rfm69_setMode(RFM69_MODE_RX);
    // todo: wait needed?
    //		rfm69_waitForModeReady();

    if (bytesRead != frameLength + 1) {
      dbg_printf("RX FIFO mismatch, got %d bytes and not %d\n", frameLength + 1, bytesRead);
    }
    return frameLength; //  bytesRead;
  }
  else
    return 0;
}

/**
 * Enable and set or disable AES hardware encryption/decryption.
 *
 * The AES encryption module will be disabled if an invalid key or key length
 * is passed to this function (aesKey = 0 or keyLength != 16).
 * Otherwise encryption will be enabled.
 *
 * The key is stored as MSB first in the RF module.
 *
 * @param aesKey Pointer to a buffer with the 16 byte AES key
 * @param keyLength Number of bytes in buffer aesKey; must be 16 bytes
 * @return State of encryption module (false = disabled; true = enabled)
 */
bool rfm69_setAESEncryption(uint8_t* aesKey, unsigned int keyLength)
{
  bool enable = false;

  // check if encryption shall be enabled or disabled
  if ((0 != aesKey) && (16 == keyLength))
    enable = true;

  // switch to standby
  rfm69_setMode(RFM69_MODE_STANDBY);

  if (true == enable)
  {
    // transfer AES key to AES key register
    rfm69_chipSelect();

    uint8_t tx[keyLength + 1];
    tx[0] = 0x3e | 0x80; /** AES MSB register */
    for (uint32_t i = 0; i < keyLength; i++) {
      tx[i+1] = aesKey[i];
    }
    // transfer key (0x3E..0x4D)
    if (spi_irq_transceive(tx, sizeof(tx), 0, 0)) {
    }
    rfm69_chipUnselect();
  }

  // set/reset AesOn Bit in packet config
  rfm69_writeRegister(0x3D, (rfm69_readRegister(0x3D) & 0xFE) | (enable ? 1 : 0));

  return enable;
}

/**
 * Wait until packet has been sent over the air or timeout.
 */
static bool rfm69_waitForPacketSent()
{
  uint64_t timeEntry = mstimer_get();
  bool success = false;
  do {
    if ((rfm69_readRegister(0x28) & 0x08) != 0) {
      success = true;
      break;
    }
    delay_ms(1);
  } while(mstimer_get() - timeEntry < TIMEOUT_PACKET_SENT);
  return success;
}

/**
 * Transmit a high or low bit in continuous mode using the external data line.
 *
 * @note Use rfm69_setDataPin() before calling this function.
 * @note Call setDataMode() before to enable continuous mode.
 *
 * @param bit true: high bit; false: low bit
 */
void rfm69_continuousBit(bool bit)
{
  // only allow this in continuous mode and if data pin was specified
  if ((RFM69_DATA_MODE_PACKET == _dataMode) || !_dataPort)
    return;

  // send low or high bit
  if (false == bit)
    gpio_clear(_dataPort, _dataPin);
  else
    gpio_set(_dataPort, _dataPin);
}

/**
 * Read the last RSSI value.
 *
 * @note Only if the last RSSI value was above the RSSI threshold, a sample can be read.
 *       Otherwise, you always get -127 dBm. Be also careful if you just switched to RX mode.
 *       You may have to wait until a RSSI sample is available.
 *
 * @return RSSI value in dBm.
 */
static int rfm69_readRSSI()
{
  _rssi = -rfm69_readRegister(0x24) / 2;

  return _rssi;
}

/**
 * Debug function to dump all RFM69 registers.
 */
void rfm69_dumpRegisters(void)
{
  for (unsigned int i = 1; i <= 0x71; i++)
  {
    dbg_printf("[0x%x]: 0x%x\n", i, rfm69_readRegister(i));
  }
}

/**
 * Enable/disable OOK modulation (On-Off-Keying).
 *
 * Default modulation is FSK.
 * The module is switched to standby mode if RX or TX was active.
 *
 * @param enable true or false
 */
void rfm69_setOOKMode(bool enable)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode)
    rfm69_setMode(RFM69_MODE_STANDBY);

  if (false == enable)
  {
    // FSK
    rfm69_writeRegister(0x02, (rfm69_readRegister(0x02) & 0xE7));
  }
  else
  {
    // OOK
    rfm69_writeRegister(0x02, (rfm69_readRegister(0x02) & 0xE7) | 0x08);
  }

  _ookEnabled = enable;
}

/**
 * Configure the data mode of the RFM69 module.
 *
 * Default data mode is 'packet'. You can choose between 'packet',
 * 'continuous with clock recovery', 'continuous without clock recovery'.
 *
 * The module is switched to standby mode if RX or TX was active.
 *
 * @param dataMode RFM69_DATA_MODE_PACKET, RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC, RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC
 */
void rfm69_setDataMode(RFM69DataMode dataMode)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _mode || RFM69_MODE_TX == _mode)
    rfm69_setMode(RFM69_MODE_STANDBY);

  switch (dataMode)
  {
  case RFM69_DATA_MODE_PACKET:
    rfm69_writeRegister(0x02, (rfm69_readRegister(0x02) & 0x1F));
    break;

  case RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC:
    rfm69_writeRegister(0x02, (rfm69_readRegister(0x02) & 0x1F) | 0x40);
    rfm69_writeRegister(0x25, 0x04); // Dio2Mapping = 01 (Data)
    rfm69_continuousBit(false);
    break;

  case RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC:
    rfm69_writeRegister(0x02, (rfm69_readRegister(0x02) & 0x1F) | 0x60);
    rfm69_writeRegister(0x25, 0x04); // Dio2Mapping = 01 (Data)
    rfm69_continuousBit(false);
    break;

  default:
    return;
  }

  _dataMode = dataMode;
}

/**
 * Set the output power level in dBm.
 *
 * This function takes care of the different PA settings of the modules.
 * Depending on the requested power output setting and the available module,
 * PA0, PA1 or PA1+PA2 is enabled.
 *
 * @param dBm Output power in dBm
 * @return 0 if dBm valid; else -1.
 */
int rfm69_setPowerDBm(int8_t dBm)
{
  /* Output power of module is from -18 dBm to +13 dBm
   * in "low" power devices, -2 dBm to +20 dBm in high power devices */
  if (dBm < -18 || dBm > 20)
    return -1;

  if (false == _highPowerDevice && dBm > 13)
    return -1;

  if (true == _highPowerDevice && dBm < -2)
    return -1;

  uint8_t powerLevel = 0;

  if (false == _highPowerDevice)
  {
    // only PA0 can be used
    powerLevel = dBm + 18;

    // enable PA0 only
    rfm69_writeRegister(0x11, 0x80 | powerLevel);
  }
  else
  {
    if (dBm >= -2 && dBm <= 13)
    {
      // use PA1 on pin PA_BOOST
      powerLevel = dBm + 18;

      // enable PA1 only
      rfm69_writeRegister(0x11, 0x40 | powerLevel);

      // disable high power settings
      _highPowerSettings = false;
      rfm69_setHighPowerSettings(_highPowerSettings);
    }
    else if (dBm > 13 && dBm <= 17)
    {
      // use PA1 and PA2 combined on pin PA_BOOST
      powerLevel = dBm + 14;

      // enable PA1+PA2
      rfm69_writeRegister(0x11, 0x60 | powerLevel);

      // disable high power settings
      _highPowerSettings = false;
      rfm69_setHighPowerSettings(_highPowerSettings);
    }
    else
    {
      // output power from 18 dBm to 20 dBm, use PA1+PA2 with high power settings
      powerLevel = dBm + 11;

      // enable PA1+PA2
      rfm69_writeRegister(0x11, 0x60 | powerLevel);

      // enable high power settings
      _highPowerSettings = true;
      rfm69_setHighPowerSettings(_highPowerSettings);
    }
  }

  return 0;
}

/**
 * Check if the channel is free using RSSI measurements.
 *
 * This function is part of the CSMA/CA algorithm.
 *
 * @return true = channel free; otherwise false.
 */
static bool rfm69_channelFree()
{
  if (rfm69_readRSSI() < CSMA_RSSI_THRESHOLD)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void rfm69_setResetPin(uint32_t resetPort, uint32_t resetPin)
{
  _resetPort = resetPort;
  _resetPin = resetPin;
  gpio_mode_setup(_resetPort, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, _resetPin);
}

void rfm69_setDataPin(uint32_t dataPort, uint32_t dataPin)
{
  _dataPort = dataPort;
  _dataPin = dataPin;
  gpio_mode_setup(_dataPort, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, _dataPin);
}

int rfm69_getRSSI()
{
  return _rssi;
}

void rfm69_setAutoReadRSSI(bool enable)
{
  _autoReadRSSI = enable;
}

void rfm69_setCSMA(bool enable)
{
  _csmaEnabled = enable;
}

/** @}
*
*/
