cmake_minimum_required(VERSION 3.13)

# Default to Raspberry Pi Pico 2 (RP2350)
if (NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

# Include Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)

# Overclock Pico 2 default clock to 250 MHz
set(PICO_DEFAULT_CPU_FREQ_MHZ 250)

project(Pico2_BMI160_Flight_Controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Executable target
add_executable(Pico2_BMI160_Flight_Controller
    main.cpp
)

# Link hardware libraries
target_link_libraries(Pico2_BMI160_Flight_Controller
    pico_stdlib
    hardware_gpio
    hardware_spi
    hardware_clocks
    hardware_uart
)

# Enable USB serial output for PC console / 3D Flight Deck
pico_enable_stdio_usb(Pico2_BMI160_Flight_Controller 1)
pico_enable_stdio_uart(Pico2_BMI160_Flight_Controller 0)

# Generate UF2, ELF, BIN, HEX outputs
pico_add_extra_outputs(Pico2_BMI160_Flight_Controller)
