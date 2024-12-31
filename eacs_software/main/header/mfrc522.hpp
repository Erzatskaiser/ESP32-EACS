#pragma once

// Custom headers
#include "./types.hpp"

// STL headers
#include <cstring>

// Vendor HAL headers
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_intr_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"

// Status codes for class methods
enum mfrc522_status : uint8_t {
  MFRC522_OK = 0x00,      /* Success code */
  MFRC522_ERR = 0x01,     /* Generic error code */
  MFRC522_COLL = 0x02,    /* Collision error code */
  MFRC522_TIMEOUT = 0x03, /* Timeout error code */
  MFRC522_FULL = 0x04,    /* Buffer is not big enough */
  MFRC522_CRC = 0x05,     /* CRC_A does not match */
  MFRC522_NAK = 0xFF,     /* PICC responded with NAK */
};

// MFRC522 Class
class MFRC522 {
 public:
  MFRC522(gpio_num_t* pins, core_num core = CORE0);
  mfrc522_status initializeChip();

 private:
  // Device and protocol constants
  const uint8_t fifo_size = 64; /* Size of FIFO buffer on chip */
  const uint8_t key_size = 6;   /* Size of Crypto1 key */
  const uint8_t mf_ack = 0xA;   /* MIFARE Classic ACK is 0xA */

  // Configuration variables
  gpio_num_t reset_pin = GPIO_NUM_0;
  gpio_num_t chipSelect_pin = GPIO_NUM_1;
  bool is_spi = true;   /* Is SPI being used (default) */
  bool is_i2c = false;  /* Is I2C being used */
  bool is_uart = false; /* Is UART being used */

  // Spinlock to enforce mutual exclusion
  portMUX_TYPE mfrc522_lock = portMUX_INITIALIZER_UNLOCKED;

  // Communication handles
  spi_device_handle_t spi_handle;

  // MFRC522 version 1.0 self-test results
  const uint8_t fifo_test_v1[64]{
      0x00, 0xC6, 0X37, 0xD5, 0x32, 0xB7, 0x57, 0x5C, 0XC2, 0xD8, 0x7C,
      0x4D, 0xD9, 0x70, 0xC7, 0x73, 0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1,
      0x3E, 0x5A, 0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E, 0x64,
      0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC, 0x22, 0xBC, 0xD3, 0x72,
      0x35, 0xCD, 0xAA, 0x41, 0x1F, 0xA7, 0xF3, 0x63, 0x14, 0xDE, 0x7E,
      0x02, 0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79};

  // MFRC522 version2.0 self-test results
  const uint8_t fifo_test_v2[64]{
      0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95, 0xD0, 0xE3, 0x0D,
      0x3D, 0x27, 0x89, 0x5C, 0xDE, 0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B,
      0x89, 0x82, 0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49, 0x7C,
      0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81, 0x5D, 0x48, 0x76, 0xD5,
      0x71, 0x61, 0x21, 0xA9, 0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B,
      0x6D, 0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F};

  // Commands of the MFRC522 ship
  enum mfrc522_commands : uint8_t {
    Idle = 0x00,             /* No actions */
    Mem = 0x01,              /* Store 25 bytes in internal buffer */
    GenerateRandomID = 0x02, /* Generate random 10-byte ID number */
    CalcCRC = 0x03,          /* Activate CRC coprocessor or perform self test */
    Transmit = 0x04,         /* Transmit data from FIFO buffer */
    NoCmdChange = 0x07,      /* No command change */
    Receive = 0x08,          /* Activate receiver circuits */
    Transceive = 0xC, /* Transmit data from FIFO and activate receiver after */
    MFAuthent = 0xE,  /* Perform Mifare standard auth as reader */
    SoftReset = 0xF,  /* Reset MFRC522 */
  };

  // Commands sent to PICC (defined by standards)
  enum picc_commands : uint8_t {

    // Manage communiation with multiple PICCs (ISO 14443-3)
    REQA = 0x26,       /* Request - Type A*/
    WUPA = 0x52,       /* Wake-up - Type A */
    AC_SEL_CL1 = 0x93, /* Anti-collision or Select, CL1 */
    HALT = 0x50,       /* Halt command - Type A */
    CascTag = 0x88,    /* Cascade Tag, used during collision */
    AC_SEL_CL2 = 0x95, /* Anti-collision or Select, CL2 */
    AC_SEL_CL3 = 0x97, /* Anti-collision or Select, CL3 */
    ATR = 0xE0,        /* Answer to Reset */

    // Commands for MIFARE Classic
    AUTH_KEY_A = 0x60, /* Authenticate with Key A */
    AUTH_KEY_B = 0x61, /* Authenticate with Key B */
    READ = 0x30,       /* Read nlock from authenticated sector */
    WRITE = 0xA0,      /* Write block from authenticated sector */
    DECREMENT = 0xC0,  /* Decrement block content */
    INCREMENT = 0xC1,  /* Increment block content */
    RESTORE = 0xC2,    /* Restore previous value */
    TRANSFER = 0xB0,   /* Move data between block and buffer*/

    // Additional commands for MIFARE Ultralight
    UL_WRITE = 0xA2 /* Writes 4 bytes */
  };

  // Values for the RFCfg register (receiver gain configuration)
  enum mfrc522_recv_gain : uint8_t {
    dB18 = 0x00,
    db23 = 0x01,
    db18_2 = 0x02,
    db23_3 = 0x3,
    db33 = 0x04,
    db38 = 0x05,
    db43 = 0x06,
    db48 = 0x07
  };

  // Registers of the MFRC522 chip
  // See 9.2 and 9.3 at https://nxp.com/docs/en/data-sheet/MFRC522.pdf
  enum mfrc522_register : uint8_t {

    // Command and status registers
    CommandReg = 0x01,
    ComIEnReg = 0x02,
    DivIEnReg = 0x03,
    ComIrqReg = 0x04,
    DivIrqReg = 0x05,
    ErrorReg = 0x06,
    Status1Reg = 0x07,
    Status2Reg = 0x08,
    FIFODataReg = 0x09,
    FIFOLevelReg = 0xA,
    WaterLevelReg = 0xB,
    ControlReg = 0xC,
    BitFramingReg = 0xD,
    CollReg = 0xE,

    // Command registers
    ModeReg = 0x11,
    TxModeReg = 0x12,
    RxModeReg = 0x13,
    TxControlReg = 0x14,
    TxASKReg = 0x15,
    TxSelReg = 0x16,
    RxSelReg = 0x17,
    RxThresholdReg = 0x18,
    DemodReg = 0x19,
    MfTxReg = 0x1C,
    MfRxReg = 0x1D,
    SerialSpeedReg = 0x1F,

    // Configuration registers
    CRCResultReg_MSB = 0x21,
    CRCResultReg_LSB = 0x22,
    ModWidthReg = 0x24,
    RFCfgReg = 0x26,
    GsNReg = 0x27,
    CWGsPReg = 0x28,
    ModGsPReg = 0x29,
    TModeReg = 0x2A,
    TPrescalerReg = 0x2B,
    TReloadReg_Hi = 0x2C,
    TReloadReg_Lo = 0x2D,
    TCounterValReg_Hi = 0x2E,
    TCounterValReg_Lo = 0x2F,

    // Test registers
    TestSel1Reg = 0x31,
    TestSel2Reg = 0x32,
    TestPinEnReg = 0x33,
    TestPinValueReg = 0x34,
    TestBusReg = 0x35,
    AutoTestReg = 0x36,
    VersionReg = 0x37,
    AnalogTestReg = 0x38,
    TestDAC1Reg = 0x39,
    TestDAC2Reg = 0x3A,
    TestADCReg = 0x3B
  };

  // Private methods
  mfrc522_status readRegister(mfrc522_register reg, uint8_t* out);
  mfrc522_status writeRegister(mfrc522_register reg, uint8_t in);
  mfrc522_status clearRegisterWithMask(mfrc522_register reg, uint8_t mask);
  mfrc522_status setRegisterWithMask(mfrc522_register reg, uint8_t mask);
};
