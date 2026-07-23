cmake_minimum_required(VERSION 3.13)

set(UGV_FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE PATH
    "Path to the rp2040_zero firmware root")
include(${UGV_FIRMWARE_ROOT}/pico_sdk_import.cmake)

project(ugv_hw_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(ugv_hw_test
  src/main.cpp
  src/command.cpp
  ${UGV_FIRMWARE_ROOT}/src/protocol.cpp
  ${UGV_FIRMWARE_ROOT}/src/motor_driver.cpp
  ${UGV_FIRMWARE_ROOT}/src/motor_command.cpp
  ${UGV_FIRMWARE_ROOT}/src/encoder.cpp
  ${UGV_FIRMWARE_ROOT}/src/encoder_pio.cpp
  ${UGV_FIRMWARE_ROOT}/src/velocity_controller.cpp
  ${UGV_FIRMWARE_ROOT}/src/command_watchdog.cpp
  ${UGV_FIRMWARE_ROOT}/src/system_state.cpp
)

pico_generate_pio_header(ugv_hw_test ${UGV_FIRMWARE_ROOT}/src/encoder_quadrature.pio)

target_include_directories(ugv_hw_test PRIVATE include ${UGV_FIRMWARE_ROOT}/include)
target_compile_options(ugv_hw_test PRIVATE -Wall -Wextra)
target_link_libraries(ugv_hw_test PRIVATE
  pico_stdlib hardware_clocks hardware_gpio hardware_pio hardware_pwm hardware_timer)

pico_enable_stdio_usb(ugv_hw_test 1)
pico_enable_stdio_uart(ugv_hw_test 0)
pico_add_extra_outputs(ugv_hw_test)
