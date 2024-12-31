// Header file
#include "../header/mfrc522.hpp"

// MFRC522 class constructor
MFRC522::MFRC522(gpio_num_t* pins, comm_mode protocol, core_num core) {
  // Store pins
  chipSelect_pin = *pins;
  reset_pin = *(pins + 1);

  // Initialize SPI bus, add device to bus
  if (protocol == SPI) {
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
        /* Handle no DMA channel available error */
      }

      // Out of memory
      else if (bus_init == ESP_ERR_NO_MEM) {
        /* Handle no memory error */
      }

      // Parameters of the SPI device
      const spi_device_interface_config_t device_conf{
          .mode = 0,
          .clock_speed_hz = 10 * 1000 * 1000,
          .spics_io_num = chipSelect_pin,
          .queue_size = 7};

      // Attach MFRC522 to SPI bus
      spi_device_handle_t mfrc522_spi;
      esp_err_t add_device =
          spi_bus_add_device(host, &device_conf, &mfrc522_spi);

      // Successfully added device, break from loop
      if (add_device == ESP_OK) break;
    }
  }

  /* Initialie I2C bus */
  /* Initialize UART bus */

  /* Initialize MFRC522 */
}
