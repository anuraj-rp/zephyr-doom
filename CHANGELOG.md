# Changelog

## [0.0.0] - 2023-10-28

### Added OR Changed
- Added GPIO keys on pins 19, 20, 21 and 4 for up, right, down and left respectively
- Add blinky thread that blinks led on pin2 every 200ms
- Device tree changes to add GPIO keys and led
- Added ISR handler for the GPIO key press

### Status
- The gpio key press prints the number of the pin pressed
- Blinky thread blinks led every 200ms

## [0.0.0] - 2023-10-19

### Added OR Changed
- Created two threads gpioTask and blink printing text from each thread.

### Status
- Compilation with multithreading works. Tested via uart printing.

## [0.0.0] - 2023-10-19

### Added OR Changed
- Added gamepad.c with zephyr kernel function calls in it. Uncommented a lot of code 
- Added target_link_libraries(external_lib INTERFACE kernel) to allow external library to find kernel symbols
    - Related Issue here - https://github.com/zephyrproject-rtos/zephyr/issues/31613

### Status
- The libraries compile with zephyr functions calls usage inside external libraries

## [0.0.0] - 2023-10-19

### Added OR Changed
- Added gamepad.c with zephyr/kernel.h includes to port it
- Added DEPEDNS zephyr_interface to the ExternalProject_Add invocation of prboom_esp32s3 lib build via ExternalProject_Add
    - Solution mentioned here - https://github.com/zephyrproject-rtos/zephyr/issues/32537

### Status
- The libraries prboom, prboom_esp32s3, prboom_wad_tables build properly with zephyr west build tool with zephyr includes needed

## [0.0.0] - 2023-10-15

### Added OR Changed
- Ported prboom-esp32-compat/i_system.c to Zephyr

### Status
- The libraries prboom, prboom_esp32s3, prboom_wad_tables build properly with zephyr west build tool 

## [0.0.0] - 2023-10-15

### Added OR Changed
- First buildable commit building the prboom-esp32-compat library
- prboom-esp32-compat was using wad files from SD card. Removed all SD card functions
  and replaced it with functions to read from SPI flash from https://github.com/anuraj-rp/esp32-doom
  The SD card version functions are here - https://github.com/anuraj-rp/doom-espidf
- commented out all FreeRTOS function calls from prboom-esp32-compat
- Excluded psxcontroller.c from build using Makefile filter-out(Thanks ChatGPT)

### Status
- The libraries prboom, prboom_esp32s3, prboom_wad_tables build properly with zephyr west build tool 

## [0.0.0] - 2023-10-15

### Added OR Changed
- The build was failing because of clashing files called md5.h/.c in components/crypto/include in espressif hal and prboom.
  Fixed it by removing the espressif_hal include path in top-level CMakeLists.txt and prepending prboom path before z_includes
- prboom linking failed because it could not find prboom-wad-tables/include path.
  Fixed it by hardcoding the path in the source files tables.c and v_video.c
- Replace mallac with k_malloc in z_zone.c

### Status
- The libraries prboom and prboom_wad_tables build properly with zephyr west build tool 

## [0.0.0] - 2023-10-12

### Added OR Changed
- Added Makefile for prboom folder and ExternalProject_Add for prboom in top-level CMakeLists.txt
- Modified Makefile to properly specify include path 

## [0.0.0] - 2023-10-12

### Added OR Changed
- Added this changelog :)
- Added Makefile and CMake External Project based lib creation based on this [Example](https://github.com/anuraj-rp/zephyr-esp32/tree/main/esp32_samples/03_mylibtest)

