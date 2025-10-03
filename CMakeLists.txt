cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(simple-bmv080-connection)

# --- Source Files and Include Directories ---

target_include_directories(app PRIVATE
    src
    lib/arm_cortex_m33f
    lib
    )

target_sources(app PRIVATE
    src/main.c
    lib/i2c.c
    )

target_link_libraries(app PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/arm_cortex_m33f/lib_bmv080.a
    ${CMAKE_SOURCE_DIR}/lib/arm_cortex_m33f/lib_postProcessor.a
    )