cmake_minimum_required(VERSION 3.13)

include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)

project(ugv_rp2040_zero C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(ugv_rp2040_zero
  src/main.cpp
  src/protocol.cpp
  src/motor_driver.cpp
  src/motor_command.cpp
  src/encoder.cpp
  src/encoder_pio.cpp
  src/velocity_controller.cpp
  src/command_watchdog.cpp
  src/system_state.cpp
)

pico_generate_pio_header(ugv_rp2040_zero ${CMAKE_CURRENT_LIST_DIR}/src/encoder_quadrature.pio)

target_include_directories(ugv_rp2040_zero PRIVATE include)
target_compile_options(ugv_rp2040_zero PRIVATE -Wall -Wextra)
target_link_libraries(ugv_rp2040_zero PRIVATE pico_stdlib hardware_clocks hardware_gpio hardware_pio hardware_pwm hardware_timer)

pico_enable_stdio_usb(ugv_rp2040_zero 1)
pico_enable_stdio_uart(ugv_rp2040_zero 0)
pico_add_extra_outputs(ugv_rp2040_zero)
