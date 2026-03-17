
add_compile_options(-Wall -Wextra -Werror)

add_executable(circuits
    circuits.cpp
    locos.cpp
)

pico_enable_stdio_uart(circuits 0)
pico_enable_stdio_usb(circuits 1)

target_link_libraries(circuits PRIVATE
    pico_stdlib
    pico_stdio_usb
    dcc
    misc
    railroad
)

pico_add_extra_outputs(circuits)
