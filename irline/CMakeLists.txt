add_executable(barcode_scanner
        barcode_scanner.c
        )

target_link_libraries(barcode_scanner pico_stdlib hardware_adc hardware_gpio pico_time pico_cyw43_arch_none)

# create map/bin/hex file etc.
pico_add_extra_outputs(barcode_scanner)

# add url via pico_set_program_url
example_auto_set_url(barcode_scanner)

pico_enable_stdio_usb(barcode_scanner 1)