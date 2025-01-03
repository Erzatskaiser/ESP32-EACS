// Header file
#include "../header/mfrc522.hpp"

#include <cstdint>

#include "esp_timer.h"

// MFRC522 class constructor
MFRC522::MFRC522(gpio_num_t* pins, core_num core) {
  // Store pins
  chipSelect_pin = *pins;
  reset_pin = *(pins + 1);

  // Configure the reset pin as an output
  const gpio_config_t reset_config{(1ULL << reset_pin), GPIO_MODE_OUTPUT,
                                   GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE,
                                   GPIO_INTR_DISABLE};
  gpio_config(&reset_config);

  // Adjust based on selected core
  esp_intr_cpu_affinity_t spi_isr = ESP_INTR_CPU_AFFINITY_0;
  if (core == CORE1) spi_isr = ESP_INTR_CPU_AFFINITY_1;

  // Available SPI hosts (SPI1 not available on all platforms)
  spi_host_device_t hosts[]{SPI1_HOST, SPI2_HOST, SPI3_HOST};

  // Iterate over all hosts (skipping SPI1)
  for (uint8_t i = 1; i < 2; i++) {
    //  Current host
    spi_host_device_t host = hosts[i];

    // Bus parameters
    const spi_bus_config_t bus_conf{.mosi_io_num = *(pins + 2),
                                    .miso_io_num = *(pins + 3),
                                    .sclk_io_num = *(pins + 4),
                                    .quadwp_io_num = -1,
                                    .quadhd_io_num = -1,
                                    .isr_cpu_id = spi_isr};

    // Initialize the bus
    esp_err_t bus_init = spi_bus_initialize(host, &bus_conf, SPI_DMA_CH_AUTO);

    // No available DMA channel
    if (bus_init == ESP_ERR_NOT_FOUND) {
      /* TODO: Handle no DMA channel available error */
    }

    // Out of memory
    else if (bus_init == ESP_ERR_NO_MEM) {
      /* TODO: Handle no memory error */
    }

    // Parameters of the SPI device
    const spi_device_interface_config_t device_conf{
        .mode = 0,
        .clock_speed_hz = 5000000,
        .spics_io_num = chipSelect_pin,
        .queue_size = 7};

    // Attach MFRC522 to SPI bus
    esp_err_t add_device = spi_bus_add_device(host, &device_conf, &spi_handle);

    // Successfully added device, break from loop
    if (add_device == ESP_OK) break;
  }
}

// idle: (none) --> (mfrc522_status)
// No action, cancel current command execution
mfrc522_status MFRC522::idle() {
  return clearRegisterWithMask(CommandReg, 0x0F);
}

// storeInternal: (none) --> (mfrc522_status)
// Trasfers FIFO data to buffer, using data already in FIFO
mfrc522_status MFRC522::storeInternal() {
  return clearAndSetRegWithMask(CommandReg, 0x0F, 0x01);
}

// storeInternal: (uint8_t*) --> (mfrc522_status)
// Takes 25 bytes of input, writes to FIFO and transfer to buffer
mfrc522_status MFRC522::storeInternal(uint8_t* data) {
  // Reset FIFO buffer pointers
  flushFIFO();

  // Write 25 bytes of data to FIFO
  writeToFIFO(data, 25);

  // Execute MEM command, return status
  return clearAndSetRegWithMask(CommandReg, 0x0F, 0x01);
}

// flushFIFO: (none) --> (mfrc522_status)
// Flush the FIFO buffer by resetting buffer pointers
mfrc522_status MFRC522::flushFIFO() {
  return writeRegister(FIFOLevelReg, 0x80);
}

// writeToFIFO: (uint8_t) --> (mfrc522_status)
// Write a byte of data to the FIFO buffer
mfrc522_status MFRC522::writeToFIFO(uint8_t data) {
  return writeRegister(FIFODataReg, data);
}

// writeToFIFO: (uint8_t*, uint8_t) --> (mfrc522_status)
// Writes data of given length to the FIFO buffer
mfrc522_status MFRC522::writeToFIFO(uint8_t* data, uint8_t length) {
  // Ensure size is not greater than buffer size
  length %= fifo_size + 1;

  // Write data to the buffer
  for (size_t i = 0; i < length; i++) {
    if (writeRegister(FIFODataReg, *data) != MFRC522_OK) return MFRC522_ERR;
    ++data;
  }

  return MFRC522_OK;
}

// readFromFIFO: (uint8_t*, uint8_t) --> (mfrc522_status)
// Reads data from the FIFO buffer and write to output variable
mfrc522_status MFRC522::readFromFIFO(uint8_t* out, uint8_t length) {
  for (size_t i = 0; i < length; i++) {
    if (readRegister(FIFODataReg, out) != MFRC522_OK) return MFRC522_ERR;
    ++out;
  }
  return MFRC522_OK;
}

// generateRandomID: (uint8_t*) --> (mfrc522_status)
// Generate a 10 byte random ID number
mfrc522_status MFRC522::generateRandomID(uint8_t* out) {
  // TODO: Generate random data and write into array
}

// calculateCRC: (uint8_t*, uint8_t*, uint8_t) --> (mfrc522_status)
// Performs CRC computations on the CRC coprocessor using user input
mfrc522_status MFRC522::calculateCRC(uint8_t* data, uint8_t* out,
                                     uint8_t length) {
  // Idle command to stop active comands
  idle();

  // Clear CRCIRq interrupt request bit
  clearRegisterWithMask(DivIrqReg, 0x04);

  // Write data to FIFO
  flushFIFO();
  writeToFIFO(data, length);

  // Start calculation using CommandReg
  setRegisterWithMask(CommandReg, 0x03);

  // Set end time
  uint32_t end_time = (esp_timer_get_time() / 1000) + 500;

  // Iterate until timeout
  while (esp_timer_get_time() <= end_time) {
    // Read data from interupt request register
    uint8_t data;
    readRegister(DivIrqReg, &data);

    // Check whether CalcCRC command is done
    if (data & 0x04) {
      // Stop command and idle
      idle();

      // Store results of CRC command
      mfrc522_status r1_op = readRegister(CRCResultReg_LSB, out);
      ++out;
      mfrc522_status r2_op = readRegister(CRCResultReg_MSB, out);

      // Update status code
      if (r1_op == MFRC522_OK && r2_op == MFRC522_OK)
        return MFRC522_OK;
      else
        return MFRC522_ERR;
    }
  }

  // Return status code
  return MFRC522_TIMEOUT;
}

// initializeChip: (none) --> (mfrc522_status)
// Initializes the MFRC522 chip
mfrc522_status MFRC522::initializeChip() {
  // Perform a hard reset
  mfrc522_status reset_status = hardReset();
  if (reset_status != MFRC522_OK) return MFRC522_ERR;

  // Initialize timer registers
  writeRegister(
      TModeReg,
      0x80); /* timer start automatically at transmission end always */
  writeRegister(TPrescalerReg, 0xA9);  /* f_timer = 40kHz */
  writeRegister(TReloadReg_Hi, 0x03);  // Reload timer has value 0x3E8 (25ms)
  writeRegister(TReloadReg_Lo, 0xE8);

  // Adjust transmit modulation settings
  setRegisterWithMask(TxASKReg, 0x40); /* 100% ASK modulation always */

  // Adjust mode sttingsn for transmitting and receiving
  clearRegisterWithMask(ModeReg,
                        0x01); /* CalCRC command has prest value 0xA671 */

  // Enable the antena driving pins
  mfrc522_status antenna_op = antennaOn();
  if (antenna_op != MFRC522_OK) return MFRC522_ERR;

  // Retun status
  return MFRC522_OK;
}

// selfTest: (none) --> (mfrc522_status)
// Performs a self test of the MFRC522 (described in 16.1.1 of datasheet)
mfrc522_status MFRC522::selfTest() {
  // Perform soft reset
  resetChip();

  // Write 0x00 to internal buffer
  uint8_t zero_arr[25] = {0x00};
  storeInternal(zero_arr);

  // Write 0x09 to AutoTestReg (enable self test)
  writeRegister(AutoTestReg, 0x09);

  // Write 0x00 to the FIFO buffer
  writeToFIFO(0x00);

  // Start self test using the CalcCRC command
  clearAndSetRegWithMask(CommandReg, 0x0F, 0x03);

  // Test is initiated, completed when FIFO has 64 bytes or timeout
  mfrc522_status status = MFRC522_TIMEOUT;
  uint32_t end_time = (esp_timer_get_time() / 1000) + 500;
  while (esp_timer_get_time() <= end_time) {
    // Read number of bytes in buffer from FIFOLevelReg
    uint8_t size;
    readRegister(FIFOLevelReg, &size);

    // If size is > 64
    size &= 0x7F;
    if (size >= 64) {
      status = MFRC522_OK;
      break;
    }
  }
  if (status == MFRC522_TIMEOUT) return status;

  // Write Idle command to stop CalCRC
  idle();

  // Read 64 bytes from FIFO buffer
  uint8_t test_result[64];
  readFromFIFO(test_result, 64);

  // Reset AutoTestReg register
  clearRegisterWithMask(AutoTestReg, 0x0F);

  // Read version register to get chip version
  uint8_t version;
  readRegister(VersionReg, &version);

  // Check which test results to use
  uint8_t reference[64];
  if (version == 0x91)
    memcpy(reference, fifo_test_v1, 64);
  else if (version == 0x92)
    memcpy(reference, fifo_test_v2, 64);

  // Compare test results
  for (size_t i = 0; i < 64; i++) {
    if (reference[i] != test_result[i]) return MFRC522_ERR;
  }

  // TODO: Reinitialize MRFC522 might be needed

  // Return status code
  return MFRC522_OK;
}

// resetChip: (none) --> (mfrc522_status)
// Perform a soft reset of the MRFC522
mfrc522_status MFRC522::resetChip() {
  // Set command bits in command register
  mfrc522_status write_op = setRegisterWithMask(CommandReg, SoftReset);

  // Wait for operation to take effect
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // Return status code
  return write_op;
}

// wakeup: (none) --> (mfrc522_status)
// Wake up the MFRC522 module from hibernation
mfrc522_status MFRC522::wakeup() {
  // Clear power down bit
  mfrc522_status status = clearRegisterWithMask(CommandReg, 0x10);
  if (status != MFRC522_OK) return MFRC522_ERR;

  /* Check that bit has been set, with timeout of 500ms */
  uint8_t bit_value = 0;
  readRegister(CommandReg, &bit_value);

  // Get end time
  uint32_t end_time = (esp_timer_get_time() / 1000) + 500;

  // Loop until timeout
  while (esp_timer_get_time() <= end_time) {
    // Read new value
    if ((bit_value & 0x10) != 0x00) {
      vTaskDelay(1 / portTICK_PERIOD_MS);
      readRegister(CommandReg, &bit_value);
    }

    // Successfull completion
    else
      return MFRC522_OK;
  }

  return MFRC522_TIMEOUT;
}

// hibernate: (none) --> (mfrc522_status)
// Hibernate the MFRC522 module (no reset on power on)
mfrc522_status MFRC522::hibernate() {
  mfrc522_status status = setRegisterWithMask(CommandReg, 0x10);
  return status;
}

// powerDown: (none) --> (mfrc522_status)
// Turn off the MFRC522 module (reset on power on)
mfrc522_status MFRC522::powerDown() {
  esp_err_t op = gpio_set_level(reset_pin, 0);
  if (op != ESP_OK) return MFRC522_ERR;
  return MFRC522_OK;
}

// adjustAntennaGain: (mfrc522_gain) --> (mfrc522_status)
// Adjusts the gain of the antenna
mfrc522_status MFRC522::adjustAntennaGain(mfrc522_gain gain) {
  // Get current antenna gain, checking for error
  mfrc522_gain out;
  mfrc522_status read_op = getAntennaGain(&out);
  if (read_op != MFRC522_OK) return MFRC522_ERR;

  // Check that desired gain is not equal to current gain
  if (gain == out) return MFRC522_OK;

  // Clear bits and set new gain value
  mfrc522_status op = clearAndSetRegWithMask(RFCfgReg, 0x70, gain << 4);

  // Return status code
  return op;
}

// getAntennaGain: (mfrc522_gain*) --> (mfrc522_status)
// Get the current gain of the antenna
mfrc522_status MFRC522::getAntennaGain(mfrc522_gain* out) {
  // Get the currentn value of the register
  uint8_t currVal = 0x00;
  mfrc522_status read_op = readRegister(RFCfgReg, &currVal);
  if (read_op != MFRC522_OK) return MFRC522_ERR;

  // Mask and shift to get gain value
  currVal &= 0x70;
  currVal >>= 4;

  // Write value to output
  *out = static_cast<mfrc522_gain>(currVal);

  // Return status code
  return MFRC522_OK;
}

// antennaOn: (none) --> (mfrc522_status)
// Enables the antenna driving pins
mfrc522_status MFRC522::antennaOn() {
  mfrc522_status result = setRegisterWithMask(TxControlReg, 0x03);
  return result;
}

// antennaOff: (none) --> (mfrc522_status)
// Disables the antenna driving pins
mfrc522_status MFRC522::antennaOff() {
  mfrc522_status result = clearRegisterWithMask(TxControlReg, 0x03);
  return result;
}

// hardReset: (none) --> (mfrc522_status)
// Hard reset the MFRC522 chip by writing the reset pin low
mfrc522_status MFRC522::hardReset() {
  // Write low for tick period
  gpio_set_level(reset_pin, 0);
  vTaskDelay(1 / portTICK_PERIOD_MS); /* datasheet specifies ~ 100ns */
  gpio_set_level(reset_pin, 1);
  vTaskDelay(1 / portTICK_PERIOD_MS); /* datasheet specified ~ 38ns for
                                       oscillator startup */

  // Read command register to ensure reset succeeded
  uint8_t curr_val;
  readRegister(CommandReg, &curr_val);

  // Return status codes
  if (curr_val != 0x20) return MFRC522_ERR;
  return MFRC522_OK;
}

// clearAndSetRegWithMask: (mfrc522_register, uint8_t, uint8_t) -->
// (mfrc522_status) Clears and set bits within a specified register using
// provided mask
mfrc522_status MFRC522::clearAndSetRegWithMask(mfrc522_register reg,
                                               uint8_t clear_mask,
                                               uint8_t set_mask) {
  mfrc522_status clear_op = clearRegisterWithMask(reg, clear_mask);
  mfrc522_status set_op = setRegisterWithMask(reg, set_mask);
  if (clear_op == MFRC522_OK && set_op == MFRC522_OK) return MFRC522_OK;
  return MFRC522_ERR;
}

// setRegisterWithMask: (mfrc522_register, uint8_t) --> (mfrc522_status)
// Sets bits within specified register using provided mask
mfrc522_status MFRC522::setRegisterWithMask(mfrc522_register reg,
                                            uint8_t mask) {
  // Obtain the current value
  uint8_t currVal = 0x00;
  mfrc522_status read_status = readRegister(reg, &currVal);
  if (read_status != MFRC522_OK) return MFRC522_ERR;

  // Apply the mask to the current value
  uint8_t newVal = currVal | mask;

  // Write the new value
  mfrc522_status write_status = writeRegister(reg, newVal);
  if (write_status != MFRC522_OK) return MFRC522_ERR;

  // Return status
  return MFRC522_OK;
}

// clearRegisterWithMask: (mfrc522_register, uint8_t*) --> (mfrc522_status)
// Clear bits within specified register using provided mask
mfrc522_status MFRC522::clearRegisterWithMask(mfrc522_register reg,
                                              uint8_t mask) {
  // Obtain the current value
  uint8_t currVal = 0x00;
  mfrc522_status read_status = readRegister(reg, &currVal);
  if (read_status != MFRC522_OK) return MFRC522_ERR;

  // Apply the mask to the current value
  uint8_t newVal = currVal & ~mask;

  // Write the new value
  mfrc522_status write_status = writeRegister(reg, newVal);
  if (write_status != MFRC522_OK) return MFRC522_ERR;

  // Return status
  return MFRC522_OK;
}

// writeRegister: (mfrc522_register, uint8_t*) --> (mfrc522_status)
// Write data in buffer into register, returning operation status
mfrc522_status MFRC522::writeRegister(mfrc522_register reg, uint8_t in) {
  // Shift address bits due to SPI protocol
  uint8_t addr = reg << 1;

  // Instantiate and zero out transaction
  spi_transaction_t write_transact;
  memset(&write_transact, 0, sizeof(write_transact));

  // Initialize first transaction (address)
  write_transact.length = 8;
  write_transact.rxlength = 0;
  write_transact.tx_buffer = &addr;

  // Transmit transaction (thread-safe)
  taskENTER_CRITICAL(&mfrc522_lock);
  esp_err_t op1 = spi_device_transmit(spi_handle, &write_transact);
  taskEXIT_CRITICAL(&mfrc522_lock);

  // Modify for second transaction (data)
  write_transact.tx_buffer = &in;

  // Transmit transaction (thread-safe)
  taskENTER_CRITICAL(&mfrc522_lock);
  esp_err_t op2 = spi_device_transmit(spi_handle, &write_transact);
  taskEXIT_CRITICAL(&mfrc522_lock);

  // Return status code
  if (op1 == ESP_OK && op2 == ESP_OK) return MFRC522_OK;
  return MFRC522_ERR;
}

// readRegister: (mfrc522_register, uint8_t*) --> (mfrc522_status)
// Reads the data in a register and writes to output pointer, returning status
mfrc522_status MFRC522::readRegister(mfrc522_register reg, uint8_t* out) {
  // Use bitwise operations to get read
  uint8_t addr = 0x80 | (reg << 1);

  // Instantiate and zero out transaction
  spi_transaction_t read_transact;
  memset(&read_transact, 0, sizeof(read_transact));

  // Initialize transaction
  uint8_t recv_buff = 0x00;
  read_transact.length = 16;
  read_transact.rxlength = 8;
  read_transact.tx_buffer = &addr;
  read_transact.rx_buffer = &addr;

  // Transmit transaction (thread-safe)
  taskENTER_CRITICAL(&mfrc522_lock);
  esp_err_t op1 = spi_device_transmit(spi_handle, &read_transact);
  taskEXIT_CRITICAL(&mfrc522_lock);

  // Write results to output
  *out = recv_buff;

  // Return error code
  if (op1 == ESP_OK) return MFRC522_OK;
  return MFRC522_ERR;
}
