
# Notes on Zephyr

  - If external linking fails due to not finding kernel related symbols add following to CMakeLists.txt
    `target_link_libraries(external_lib INTERFACE kernel)`
  - Related GitHub Issue - https://github.com/zephyrproject-rtos/zephyr/issues/31613

## [0.0.0] - 2023-10-19
  - External lib does not build with error `syscall_list.h` - No such file or directory
      - Add DEPEDNS zephyr_interface to the ExternalProject_Add invocation
      - Solution mentioned here - https://github.com/zephyrproject-rtos/zephyr/issues/32537

## [0.0.0] - 2023-10-12
  - CMakeLists.txt based build fails because of long file names. 
  - CMake external project zephyr - https://github.com/zephyrproject-rtos/zephyr/discussions/43650
