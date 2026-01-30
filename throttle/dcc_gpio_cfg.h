#pragma once

#include "hardware/uart.h"

//                     +-----| USB |-----+
//  (ts)  SDA X     D0 | 1            40 | VBUS_OUT
//  (ts)  SCL X     D1 | 2            39 | VSYS_IO
//                 GND | 3            38 | GND
//  (ts)  RST       D2 | 4            37 | 3V3_EN
//  (ts)  INT       D3 | 5            36 | 3V3_OUT
//  (fb) MISO       D4 | 6            35 | AREF
//  (fb)   CS       D5 | 7            34 | A2/D28 X CS   (dcc)
//                 GND | 8            33 | GND
//  (fb)  SCK       D6 | 9            32 | A1/D27        (11)
//  (fb) MOSI       D7 | 10           31 | A0/D26        (10)   CS (dcc)
//  (fb)   CD       D8 | 11           30 | RUN
//  (fb)  RST       D9 | 12           29 | D22           (9)
//                 GND | 13           28 | GND
// (dcc)  PWR X    D10 | 14           27 | D21           (8)    SCL (ts)
// (dcc)  SIG X    D11 | 15           26 | D20           (7)    SDA (ts)
//  (fb)  LED      D12 | 16           25 | D19           (6)    SIG (dcc)
// (dcc)  RXD X    D13 | 17           24 | D18           (5)    PWR (dcc)
//                 GND | 18           23 | GND
//  (1)            D14 | 19           22 | D17           (4)    RXD (dcc)
//  (2)            D15 | 20           21 | D16           (3)
//                     +-----------------+

constexpr int dcc_sig_gpio = 19;          // PH
constexpr int dcc_pwr_gpio = 18;          // EN
constexpr int dcc_adc_gpio = 26;          // CS (ADC0)
constexpr int dcc_rcom_gpio = 17;         // Railcom
uart_inst_t *const dcc_rcom_uart = uart0; // uart0

constexpr int dcc_dbg_rcom_read_gpio = -1;
constexpr int dcc_dbg_rcom_junk_gpio = -1;
constexpr int dcc_dbg_rcom_short_gpio = -1;
constexpr int dcc_dbg_bitstream_next_bit_gpio = -1;
constexpr int dcc_dbg_command_get_packet_gpio = -1;
