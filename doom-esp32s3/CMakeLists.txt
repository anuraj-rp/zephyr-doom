cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(doom-esp32s3)

target_sources(app PRIVATE src/main.c)

#export flags to external lib

zephyr_get_include_directories_for_lang_as_string(          C z_includes)
zephyr_get_system_include_directories_for_lang_as_string(   C z_system_includes)
zephyr_get_compile_definitions_for_lang_as_string(          C z_compile_definitions)
zephyr_get_compile_options_for_lang_as_string(              C z_compile_options)

set(external_project_cflags
    "${z_includes} ${z_system_includes} ${z_compile_definitions} ${z_compile_options}"
)

# add_subdirectory(${CMAKE_SOURCE_DIR}/prboom-wad-tables) 
# add_subdirectory(${CMAKE_SOURCE_DIR}/prboom)

include(ExternalProject)

set(prboom_wad_src_dir ${CMAKE_CURRENT_SOURCE_DIR}/prboom-wad-tables)
set(prboom_wad_build_dir ${CMAKE_CURRENT_BINARY_DIR}/prboom-wad-tables)

set(PRBOOM_WAD_DIR ${prboom_wad_build_dir}/lib)
set(PRBOOM_INCLUDE_DIR ${prboom_wad_src_dir}/include)

if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
# https://www.gnu.org/software/make/manual/html_node/MAKE-Variable.html
set(submake "$(MAKE)")
else() # Obviously no MAKEFLAGS. Let's hope a "make" can be found somewhere.
set(submake "make")
endif()

ExternalProject_Add(
    prboom_wad
    PREFIX ${prboom_wad_build_dir}
    SOURCE_DIR ${prboom_wad_src_dir}
    BINARY_DIR ${prboom_wad_src_dir}
    DOWNLOAD_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND
    ${submake}
    PREFIX=${prboom_wad_build_dir}
    CC=${CMAKE_C_COMPILER}
    AR=${CMAKE_AR}
    CFLAGS=${external_project_cflags}
    INSTALL_COMMAND ""      # This particular build system has no install command
    BUILD_BYPRODUCTS ${PRBOOM_WAD_DIR}/libprboom_wad_tables.a    
)

add_library(prboom_wad_lib STATIC IMPORTED GLOBAL)
add_dependencies(
    prboom_wad_lib
    prboom_wad
)

set_target_properties(prboom_wad_lib PROPERTIES IMPORTED_LOCATION ${PRBOOM_WAD_DIR}/libprboom_wad_tables.a)
set_target_properties(prboom_wad_lib PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${PRBOOM_INCLUDE_DIR})

target_link_libraries(app PUBLIC prboom_wad_lib)

