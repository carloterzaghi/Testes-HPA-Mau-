# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)


project(Teste-HPA C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(Teste-HPA
    Teste-HPA.cpp
    src/sx1262.cpp
    src/mpu6050.cpp
    src/tfluna.cpp
)
pico_set_program_name(Teste-HPA "Teste-HPA")
pico_set_program_version(Teste-HPA "0.1")

# Saída via USB (monitor serial) — UART desabilitada
pico_enable_stdio_uart(Teste-HPA 0)
pico_enable_stdio_usb(Teste-HPA 1)

# Bibliotecas necessárias
target_link_libraries(Teste-HPA
    pico_stdlib
    hardware_i2c
    hardware_spi
    hardware_gpio
)

# Inclui o diretório atual para encontrar sx1262.h
target_include_directories(Teste-HPA PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
)

pico_add_extra_outputs(Teste-HPA)
