cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(doom-esp32s3)

#export flags to external lib

zephyr_get_include_directories_for_lang_as_string(          C z_includes)
zephyr_get_system_include_directories_for_lang_as_string(   C z_system_includes)
zephyr_get_compile_definitions_for_lang_as_string(          C z_compile_definitions)
zephyr_get_compile_options_for_lang_as_string(              C z_compile_options)

add_subdirectory(${CMAKE_SOURCE_DIR}/prboom-wad-tables) 
add_subdirectory(${CMAKE_SOURCE_DIR}/prboom)


target_sources(app PRIVATE src/main.c)

