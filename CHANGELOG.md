# Changelog

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

