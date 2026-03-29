#pragma once

#include "hardware/uart.h"

//               +-----| USB |-----+
//  (t) T0A   D0 | 1            40 | VBUS_OUT
//  (t) T0B   D1 | 2            39 | VSYS_IO
//           GND | 3            38 | GND
//  (t) T1A   D2 | 4            37 | 3V3_EN
//  (t) T1B   D3 | 5            36 | 3V3_OUT
//  (t) TP    D4 | 6            35 | AREF
//            D5 | 7            34 | A2/D28
//           GND | 8            33 | GND
//  (s) S1    D6 | 9            32 | A1/D27
//  (s) S2    D7 | 10           31 | A0/D26  CS (dcc)
//  (s) S3    D8 | 11           30 | RUN
//  (s) S4    D9 | 12           29 | D22
//           GND | 13           28 | GND
//  (s) S5   D10 | 14           27 | D21
//  (s) S6   D11 | 15           26 | D20
//  (s) S7   D12 | 16           25 | D19     SIG (dcc)
//  (s) S0   D13 | 17           24 | D18     PWR (dcc)
//           GND | 18           23 | GND
//           D14 | 19           22 | D17     RXD (dcc)
//           D15 | 20           21 | D16
//               +-----------------+

constexpr int dcc_sig_gpio = 19;          // PH - PWM slice 1 channel B
constexpr int dcc_pwr_gpio = 18;          // EN - PWM slice 1 channel A
constexpr int dcc_adc_gpio = 26;          // CS (ADC0)
constexpr int dcc_rcom_gpio = 17;         // Railcom
uart_inst_t *const dcc_rcom_uart = uart0; // uart0

constexpr int dcc_dbg_rcom_read_gpio = -1;
constexpr int dcc_dbg_rcom_junk_gpio = -1;
constexpr int dcc_dbg_rcom_short_gpio = -1;
constexpr int dcc_dbg_bitstream_next_bit_gpio = -1;
constexpr int dcc_dbg_command_get_packet_gpio = -1;

constexpr int t0a_gpio = 0;
constexpr int t0b_gpio = 1;
constexpr int t1a_gpio = 2;
constexpr int t1b_gpio = 3;
constexpr int tp_gpio = 4; // PWM slice 2 channel A

constexpr int s0_gpio = 13;
constexpr int s1_gpio = 6;
constexpr int s2_gpio = 7;
constexpr int s3_gpio = 8;
constexpr int s4_gpio = 9;
constexpr int s5_gpio = 10;
constexpr int s6_gpio = 11;
constexpr int s7_gpio = 12;
