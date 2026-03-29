
add_compile_options(-Wall -Wextra -Werror)

add_executable(stops
    stops.cpp
)

pico_enable_stdio_uart(stops 0)
pico_enable_stdio_usb(stops 1)

target_link_libraries(stops PRIVATE
    pico_stdlib
    pico_stdio_usb
    dcc
    misc
    railroad
)

pico_add_extra_outputs(stops)

