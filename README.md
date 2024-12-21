# **ESP32 Electronic access control system (EACS)**

This project implements an access control system using the ESP32 SoC and the MFRC522 RFID reader/writer. At a high level, the system works as follows: users scan their access cards, and the associated
ID is compared against a local ID store. If the ID is not found in this local store, it is updated based on some predefined online store. Further comparison then occurs, determining whether or not the
user is granted access or not. The device keeps track of access history, which it publishes to the cloud every 12 hours. The repository contains the schematics and board layout of the device
(under _eacs_hardware_), as well as the required software (under _eacs_software_).

## Steps to run
Assuming appropriate hardware is available, the project can be built and flashed to the microcontroller by following these steps:
1. Setup a local instance of ESP-IDF using the steps outlined [here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html) (Linux/MacOS, stop at Step 5) or
   [here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/windows-setup.html) (Windows, stop before _First Steps on ESP-IDF_)
   
3. Clone the repository into a directory of choice using ```git clone https://github.com/Erzatskaiser/ESP32-EACS```
4. Navigate to the software subdirectory using ```cd eacs_software```
5. Set the development target to ESP32 using ```idf.py set-target esp32```
6. Build the project using ```idf.py build```
7. Flash the project to the ESP32 using ```idf.py -p PORT flash```
8. To observe any serial output from the ESP32, use ```idf.py -p PORT monitor```
