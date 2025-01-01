// Header file
#include "../header/mfrc522.hpp"

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
// Performs a self test of the MFRC522
mfrc522_status MFRC522::selfTest() {
  // Perform soft reset
  // Clear internal buffer, implement Config command
  // Enable self test - write 0x09 to the AutoTestReg register
  // Write 0x00 to the FIFO buffer
  // Start test with CalcCRC command
  // After test complete, read 64 bytes from FIFO buffer
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

  return MFRC522_ERR;
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

  // Clear register bits first
  mfrc522_status clear_op = clearRegisterWithMask(RFCfgReg, 0x70);
  if (clear_op != MFRC522_OK) return MFRC522_ERR;

  // Make mask, and set value
  uint8_t mask = gain << 4;
  mfrc522_status set_op = setRegisterWithMask(RFCfgReg, mask);
  if (set_op != MFRC522_OK) return MFRC522_ERR;

  // Return status code
  return MFRC522_OK;
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

// setRegisterWithMask: (mfrc522_register, uint8_t*) --> (mfrc522_status)
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
