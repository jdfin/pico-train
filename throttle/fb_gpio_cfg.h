#pragma once

#include "hardware/spi.h"

#if 1

//               +------| USB |------+
//             1 | D0       VBUS_OUT | 40
//             2 | D1        VSYS_IO | 39
//             3 | GND           GND | 38
//             4 | D2         3V3_EN | 37
//             5 | D3        3V3_OUT | 36
//  (ts) SDA   6 | D4           AREF | 35
//  (ts) SCL   7 | D5         A2/D28 | 34
//             8 | GND           GND | 33
//  (ts) RST   9 | D6         A1/D27 | 32
//  (ts) INT  10 | D7         A0/D26 | 31
//            11 | D8            RUN | 30
//            12 | D9            D22 | 29  LED  (fb)
//            13 | GND           GND | 28
//            14 | D10           D21 | 27  RST  (fb)
//            15 | D11           D20 | 26  CD   (fb)
//            16 | D12           D19 | 25  MOSI (fb)
//            17 | D13           D18 | 24  SCK  (fb)
//            18 | GND           GND | 23
//            19 | D14           D17 | 22  CS   (fb)
//            20 | D15           D16 | 21  MISO (fb)
//               +-------------------+

constexpr int fb_spi_miso_gpio = 16;
constexpr int fb_spi_mosi_gpio = 19;
constexpr int fb_spi_clk_gpio = 18;
constexpr int fb_spi_cs_gpio = 17;
spi_inst_t* const fb_spi_inst = spi0;

constexpr int fb_cd_gpio = 20;
constexpr int fb_rst_gpio = 21;
constexpr int fb_led_gpio = 22;

#else

//               +------| USB |------+
//  (ts) SDA   1 | D0       VBUS_OUT | 40
//  (ts) SCL   2 | D1        VSYS_IO | 39
//             3 | GND           GND | 38
//  (ts) RST   4 | D2         3V3_EN | 37
//  (ts) INT   5 | D3        3V3_OUT | 36
//  (fb) MISO  6 | D4           AREF | 35
//  (fb) CS    7 | D5         A2/D28 | 34  CS   (dcc)
//             8 | GND           GND | 33
//  (fb) SCK   9 | D6         A1/D27 | 32  11
//  (fb) MOSI 10 | D7         A0/D26 | 31  10
//  (fb) CD   11 | D8            RUN | 30
//  (fb) RST  12 | D9            D22 | 29  9
//            13 | GND           GND | 28
// (dcc) PWR  14 | D10           D21 | 27  8
// (dcc) SIG  15 | D11           D20 | 26  7
//  (fb) LED  16 | D12           D19 | 25  6
// (dcc) RXD  17 | D13           D18 | 24  5
//            18 | GND           GND | 23
//         1  19 | D14           D17 | 22  4
//         2  20 | D15           D16 | 21  3
//               +-------------------+

constexpr int fb_spi_miso_gpio = 4;
constexpr int fb_spi_mosi_gpio = 7;
constexpr int fb_spi_clk_gpio = 6;
constexpr int fb_spi_cs_gpio = 5;
spi_inst_t* const fb_spi_inst = spi0;

constexpr int fb_cd_gpio = 8;
constexpr int fb_rst_gpio = 9;
constexpr int fb_led_gpio = 12;

#endif
