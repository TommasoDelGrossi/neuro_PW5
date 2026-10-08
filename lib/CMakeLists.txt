cmake_minimum_required(VERSION 3.10)

# Libreria di medio livello
add_library(p24_stimulator STATIC
    src/smpt_ml_client.c
    src/smpt_ml_definitions_data_types.c
    src/smpt_ml_packet_client.c
    src/smpt_ml_packet_server.c
    src/smpt_ml_packet_utils.c
    src/smpt_ml_packet_validity.c
)

# header della liberia. Public fa sì che siano accessibili a chiunque carichi questa libreria
target_include_directories(p24_stimulator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/general
    ${CMAKE_CURRENT_SOURCE_DIR}/include/general/packet
    ${CMAKE_CURRENT_SOURCE_DIR}/include/general/packet_input_buffer
    ${CMAKE_CURRENT_SOURCE_DIR}/include/general/packet_output_buffer
    ${CMAKE_CURRENT_SOURCE_DIR}/include/general/serial_port
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mid-level
    ${CMAKE_CURRENT_SOURCE_DIR}/include/low-level
)

# Libreria precompliata di basso livello
add_library(libsmpt STATIC IMPORTED)
set_target_properties(libsmpt PROPERTIES 
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_HOST_SYSTEM_PROCESSOR}/libsmpt.a"
)

# uniamo tutto in una libreria statica
target_link_libraries(p24_stimulator PUBLIC libsmpt)