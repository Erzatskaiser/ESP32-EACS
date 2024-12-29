#pragma once

// HAL imports
#include "driver/spi_common.h"

class MFRC522 {
 public:
  MFRC522();

 private:
  const uint8_t fifo_size = 64;   // Size of FIFO buffer on chip
  const uint8_t unused_pin = -1;  // Representation for unused pin
  bool is_spi = true;             // Is SPI being used (default)
  bool is_i2c = false;            // Is I2C being used

  // Shift addresses to obtain SPI address
  uint8_t get_spi_address(uint8_t address);

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

  // Registers of the MFRC522 chip
  enum mfrc522_registers : uint8_t {

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
};
