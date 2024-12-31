// Header file
#include "../header/mfrc522.hpp"

// MFRC522 class constructor
MFRC522::MFRC522(gpio_num_t* pins, core_num core) {
  // Store pins
  chipSelect_pin = *pins;
  reset_pin = *(pins + 1);

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
        .clock_speed_hz = 10 * 1000 * 1000,
        .spics_io_num = chipSelect_pin,
        .queue_size = 7};

    // Attach MFRC522 to SPI bus
    esp_err_t add_device = spi_bus_add_device(host, &device_conf, &spi_handle);

    // Successfully added device, break from loop
    if (add_device == ESP_OK) {
      /* Write the CS line HIGH */
      break;
    };
  }
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

  // Transmit the transaction (thread-safe)
  taskENTER_CRITICAL(&mfrc522_lock);
  esp_err_t op1 = spi_device_transmit(spi_handle, &read_transact);
  taskEXIT_CRITICAL(&mfrc522_lock);

  // Write results to output
  *out = recv_buff;

  // Return error code
  if (op1 == ESP_OK) return MFRC522_OK;
  return MFRC522_ERR;
}
