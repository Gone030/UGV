# Minimal Pico SDK locator. Set PICO_SDK_PATH to an installed pico-sdk checkout.
if(NOT PICO_SDK_PATH)
  if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
  endif()
endif()

if(NOT PICO_SDK_PATH OR NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
  message(FATAL_ERROR "Pico SDK not found. Set PICO_SDK_PATH to the pico-sdk directory.")
endif()

include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
