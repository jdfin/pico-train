
add_compile_options(-Wall -Wextra -Werror)

add_executable(speeds
    speeds.cpp
)

pico_enable_stdio_uart(speeds 0)
pico_enable_stdio_usb(speeds 1)

target_link_libraries(speeds PRIVATE
    pico_stdlib
    pico_stdio_usb
    dcc
    misc
    railroad
)

pico_add_extra_outputs(speeds)
