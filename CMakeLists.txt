# Find Zephyr. This also loads Zephyr's build system.
cmake_minimum_required(VERSION 3.13.1)

set(BOARD nrf52840dk_nrf52840)
# set(BOARD nrf52833dk_nrf52833)
set(DTC_OVERLAY_FILE overlay.dts)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(draeger-module_lora_ble_scan)


# Add your source file to the "app" target. This must come after
# find_package(Zephyr) which defines the target.
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/UART_handler.c)
target_sources(app PRIVATE src/SEGGER_RTT_printf.c)

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
