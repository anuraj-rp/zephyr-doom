FILE(GLOB prboom_sources ./*.c)
#target_sources(prboom PRIVATE ${prboom_sources})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}         \
    -Wno-error=char-subscripts              \
    -Wno-error=unused-value                 \ 
    -Wno-error=parentheses                  \ 
    -Wno-error=int-to-pointer-cast          \  
    -Wno-error=unused-but-set-parameter     \ 
    -Wno-error=maybe-uninitialized          \
    -Wno-pointer-sign                       \
    -Wno-int-to-pointer-cast                \
    "
)

add_library(prboom STATIC ${prboom_sources})
target_link_libraries(prboom PUBLIC prboom_wad_tables)

target_include_directories(prboom PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(prboom_wad_tables 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${z_system_includes} 
    ../../../modules/hal/espressif/components/esp_common/include 
)

target_compile_definitions(prboom_wad_tables
    PUBLIC ${z_compile_definitions}
)

target_compile_options(prboom_wad_tables
    PUBLIC ${z_compile_options}
)