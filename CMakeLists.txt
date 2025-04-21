
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ProjetoIntegrado C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ProjetoIntegrado
  ProjetoIntegrado.c
  inc/ssd1306.c
)

pico_generate_pio_header(ProjetoIntegrado ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)


# todas as libs numa só invocação:
target_link_libraries(ProjetoIntegrado
  pico_stdlib
  hardware_i2c
  hardware_pwm
  hardware_irq
  hardware_adc
  hardware_timer     # para get_absolute_time()
  hardware_pio
)

target_include_directories(ProjetoIntegrado PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

pico_enable_stdio_uart(ProjetoIntegrado 1)
pico_enable_stdio_usb(ProjetoIntegrado 1)
pico_add_extra_outputs(ProjetoIntegrado)
