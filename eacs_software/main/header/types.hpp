#pragma once

// STL libraries
#include <cstdint>

// Communication mode
enum comm_mode : uint8_t { SPI = 0x00, i2c = 0x01, UART = 0x02 };

// Core number
enum core_num : uint8_t { CORE0 = 0x00, CORE1 = 0x01 };
