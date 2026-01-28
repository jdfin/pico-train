#pragma once

#include "hardware/i2c.h"

//                     +-----| USB |-----+
//  (ts)  SDA       D0 | 1            40 | VBUS_OUT
//  (ts)  SCL       D1 | 2            39 | VSYS_IO
//                 GND | 3            38 | GND
//  (ts)  RST       D2 | 4            37 | 3V3_EN
//  (ts)  INT       D3 | 5            36 | 3V3_OUT
//  (fb) MISO       D4 | 6            35 | AREF
//  (fb)   CS       D5 | 7            34 | A2/D28   CS   (dcc)
//                 GND | 8            33 | GND
//  (fb)  SCK       D6 | 9            32 | A1/D27        (11)
//  (fb) MOSI       D7 | 10           31 | A0/D26        (10)
//  (fb)   CD       D8 | 11           30 | RUN
//  (fb)  RST       D9 | 12           29 | D22           (9)
//                 GND | 13           28 | GND 
// (dcc)  PWR      D10 | 14           27 | D21           (8)
// (dcc)  SIG      D11 | 15           26 | D20           (7)
//  (fb)  LED      D12 | 16           25 | D19           (6)
// (dcc)  RXD      D13 | 17           24 | D18           (5)
//                 GND | 18           23 | GND 
//  (1)            D14 | 19           22 | D17           (4)
//  (2)            D15 | 20           21 | D16           (3)
//                     +-----------------+

constexpr int ts_i2c_sda_gpio = 0;
constexpr int ts_i2c_scl_gpio = 1;
i2c_inst_t* const ts_i2c_inst = i2c0;

constexpr int ts_rst_gpio = 2;
constexpr int ts_int_gpio = 3;
