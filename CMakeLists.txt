# Specify the minimum version of CMake required.
cmake_minimum_required(VERSION 3.20.0)

# Find the Zephyr package. This line is essential for any Zephyr application.
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Define the project name.
project(simple-bmv080-connection)

# --- Source Files and Include Directories ---

# Add the application's include directories.
# This tells the compiler to look in the 'lib' and 'src' folders for header files.
# Using target_include_directories is the modern, preferred method.
target_include_directories(app PRIVATE
    src
    lib/arm_cortex_m33f
    lib
    )

# Add the source files to the 'app' target.
# It is best practice to list all source files explicitly.
target_sources(app PRIVATE
    src/main.c
    lib/i2c.c
    )

# --- Link Libraries ---
# CRITICAL: Link the pre-compiled Bosch .a libraries to your application.
# Use ${CMAKE_SOURCE_DIR} to provide a full, absolute path to the files.
target_link_libraries(app PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/arm_cortex_m33f/lib_bmv080.a
    ${CMAKE_SOURCE_DIR}/lib/arm_cortex_m33f/lib_postProcessor.a
    )