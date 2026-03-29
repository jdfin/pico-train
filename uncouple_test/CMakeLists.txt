
add_compile_options(-Wall -Wextra -Werror)

add_executable(uncouple_test
    uncouple_test.cpp
)

pico_enable_stdio_uart(uncouple_test 0)
pico_enable_stdio_usb(uncouple_test 1)

target_link_libraries(uncouple_test PRIVATE
    pico_stdlib
    pico_stdio_usb
    dcc
    misc
    railroad
)

pico_add_extra_outputs(uncouple_test)

