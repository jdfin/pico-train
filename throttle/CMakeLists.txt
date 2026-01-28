
add_compile_options(-Wall -Wextra -Werror)

add_executable(throttle
    throttle.cpp
)

pico_enable_stdio_uart(throttle 0)
pico_enable_stdio_usb(throttle 1)

target_link_libraries(throttle PRIVATE
    pico_stdlib
    pico_stdio_usb
    framebuffer
    touchscreen
    gui
    misc
)

pico_add_extra_outputs(throttle)

