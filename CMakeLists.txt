
cmake_minimum_required(VERSION 3.10)

project(neuro_PW5 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# Output executable in the project root
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

# System files located in the parent directory
set(SYSTEM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

# P24 stimulator library
add_subdirectory(lib)

# Main executable
add_executable(main
    src/main.cpp
    src/utils_exofes.cpp
    src/data_logger.cpp
    src/timer.cpp
    src/grav_model.cpp

    ${SYSTEM_DIR}/utils.c
)

# Include directories
target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SYSTEM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/include
)

# Compilation options
target_compile_options(main PRIVATE -Wall -O2)

# Link libraries
target_link_libraries(main PRIVATE
    Threads::Threads
    Eigen3::Eigen
    rt
    m
    p24_stimulator 
)
