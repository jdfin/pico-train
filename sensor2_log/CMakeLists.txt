
add_compile_options(-Wall -Wextra -Werror)

add_executable(sensor2_log
    sensor2_log.cpp
)

pico_enable_stdio_uart(sensor2_log 0)
pico_enable_stdio_usb(sensor2_log 1)

target_link_libraries(sensor2_log PRIVATE
    pico_stdlib
    pico_stdio_usb
    dcc
    misc
    railroad
)

pico_add_extra_outputs(sensor2_log)

