
#include <cassert>
#include <cstdint>
#include <cstdio>
// pico
#include "hardware/spi.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
// misc
#include "argv.h"
#include "i2c_dev.h"
#include "sys_led.h"
// framebuffer
#include "color.h"
#include "font.h"
#include "pixel_565.h"
#include "pixel_image.h"
#include "roboto.h"
#include "ws35.h"
// touchscreen
#include "gt911.h"
// gui
#include "gui.h"
// dcc
#include "dcc.h"
//
#include "dcc_gpio_cfg.h"
#include "fb_gpio_cfg.h"
#include "ts_gpio_cfg.h"

using HAlign = Framebuffer::HAlign;
using Event = Touchscreen::Event;

// framebuffer

static const int fb_spi_baud_request = 15'000'000;
static int fb_spi_baud_actual = 0;

static const int work_bytes = 256;
static uint8_t work[work_bytes];

static Ws35 fb(fb_spi_inst, fb_spi_miso_gpio, fb_spi_mosi_gpio, fb_spi_clk_gpio,
               fb_spi_cs_gpio, fb_spi_baud_request, fb_cd_gpio, fb_rst_gpio,
               fb_led_gpio, 480, 320, work, work_bytes);

// touchscreen

static const uint ts_i2c_baud_request = 400'000;
static uint ts_i2c_baud_actual = 0;
static const uint8_t ts_i2c_addr = 0x14; // 0x14 or 0x5d

static I2cDev i2c_dev(ts_i2c_inst, ts_i2c_scl_gpio, ts_i2c_sda_gpio,
                      ts_i2c_baud_request);

static Gt911 ts(i2c_dev, ts_i2c_addr, ts_rst_gpio, ts_int_gpio);

//////////////////////////////////////////////////////////////////////////////
///// DCC ////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

static DccAdc adc(dcc_adc_gpio);

static DccCommand command(dcc_sig_gpio, dcc_pwr_gpio, -1, adc, dcc_rcom_uart,
                          dcc_rcom_gpio);

static DccThrottle *throttle = nullptr;

//////////////////////////////////////////////////////////////////////////////
// GUI ///////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

static constexpr Color screen_bg = Color::white();
static constexpr Color screen_fg = Color::black();

// fb.width() is not constexpr
static constexpr int fb_width = 480;
static constexpr int fb_height = 320;

static constexpr int fb_width2 = fb_width / 2;

//DIGIT_IMAGE_ARRAY(roboto_28, screen_fg, screen_bg); // roboto_28_digit_img
DIGIT_IMAGE_ARRAY(roboto_36, screen_fg, screen_bg); // roboto_36_digit_img
DIGIT_IMAGE_ARRAY(roboto_48, screen_fg, screen_bg); // roboto_48_digit_img

static constexpr Color btn_up_bg = Color::gray(95);
static constexpr Color btn_dn_bg = Color::gray(85);

//////////////////////////////////////////////////////////////////////////////
///// Number Pad /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// This is used for more that one page.

namespace NumPad {

// +------------------------------------+
// | [MAIN] [LOCO] [FUNC] [PROG] [MORE] |
// |                                    |
// |                  [ 7 ] [ 8 ] [ 9 ] |
// |                                    |
// |                  [ 4 ] [ 5 ] [ 6 ] |
// |                                    |
// |                  [ 1 ] [ 2 ] [ 3 ] |
// |                                    |
// |                  [BS ] [ 0 ] [CLR] |
// +------------------------------------+

static constexpr int btn_wid = 64;
static constexpr int btn_hgt = 64;
static constexpr int btn_spc = 1;
static constexpr int btn_brd = 2;

static constexpr int btn_c3 = fb_width - 1 - btn_wid;
static constexpr int btn_c2 = btn_c3 - btn_spc - btn_wid;
static constexpr int btn_c1 = btn_c2 - btn_spc - btn_wid;

static constexpr int btn_r1 = 60;
static constexpr int btn_r2 = btn_r1 + btn_hgt + btn_spc;
static constexpr int btn_r3 = btn_r2 + btn_hgt + btn_spc;
static constexpr int btn_r4 = btn_r3 + btn_hgt + btn_spc;

static constexpr Font num_font = roboto_36;
static constexpr Font txt_font = roboto_28;

static void update_num(GuiNumber &num, int val, int min_val, int max_val);

static void btn_dn(intptr_t);

// digits 0-9

#define PAD_BTN(COL, ROW, NUM)                                               \
    static constexpr PixelImage<Pixel565, btn_wid, btn_hgt>                  \
        pad_##NUM##_btn_up_img = label_img<Pixel565, btn_wid, btn_hgt> /**/  \
        (#NUM, num_font, screen_fg, btn_brd, screen_fg, btn_up_bg);          \
                                                                             \
    static constexpr PixelImage<Pixel565, btn_wid, btn_hgt>                  \
        pad_##NUM##_btn_dn_img = label_img<Pixel565, btn_wid, btn_hgt> /**/  \
        (#NUM, num_font, screen_fg, btn_brd, screen_fg, btn_dn_bg);          \
                                                                             \
    static GuiButton pad_##NUM##_btn(                                        \
        fb, COL, ROW, screen_bg, &pad_##NUM##_btn_up_img.hdr, /* enabled */  \
        &pad_##NUM##_btn_up_img.hdr,                          /* disabled */ \
        &pad_##NUM##_btn_dn_img.hdr,                          /* pressed */  \
        nullptr, 0,                                           /* on_click */ \
        btn_dn, NUM,                                          /* on_down */  \
        nullptr, 0)                                           /* on_up */

PAD_BTN(btn_c1, btn_r1, 7);
PAD_BTN(btn_c2, btn_r1, 8);
PAD_BTN(btn_c3, btn_r1, 9);
PAD_BTN(btn_c1, btn_r2, 4);
PAD_BTN(btn_c2, btn_r2, 5);
PAD_BTN(btn_c3, btn_r2, 6);
PAD_BTN(btn_c1, btn_r3, 1);
PAD_BTN(btn_c2, btn_r3, 2);
PAD_BTN(btn_c3, btn_r3, 3);
// BS in C1 R4
PAD_BTN(btn_c2, btn_r4, 0);
// CLR in C3 R4

// backspace and clear buttons

// callback is passed a digit 0-9, or one of these
static constexpr int btn_bs_val = -1;
static constexpr int btn_clr_val = -2;

BUTTON_1(pad_bs, "BS", fb, btn_c1, btn_r4, btn_wid, btn_hgt, btn_brd, txt_font,
         screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,                                 // on_click
         btn_dn, btn_bs_val,                         // on_down
         nullptr, 0,                                 // on_up
         GuiButton::Mode::Momentary, false);

BUTTON_1(pad_clr, "CLR", fb, btn_c3, btn_r4, btn_wid, btn_hgt, btn_brd,
         txt_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,                                           // on_click
         btn_dn, btn_clr_val,                                  // on_down
         nullptr, 0,                                           // on_up
         GuiButton::Mode::Momentary, false);

} // namespace NumPad

//////////////////////////////////////////////////////////////////////////////
///// MAIN ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// +------------------------------------+
// | [MAIN] [LOCO] [FUNC] [PROG] [MORE] |
// |                                    |
// | [ Horn ]                  [Lights] |
// |                  3                 |
// | [ Bell ]                  [Engine] |
// |                                    |
// | Req [Reverse] [Stop] [Forward] Act |
// |                                    |
// | <<<<<<<<<< Speed Slider >>>>>>>>>> |
// +------------------------------------+

namespace MainPage {

///// Loco Number

static constexpr int loco_col = fb_width / 2;
static constexpr int loco_row = fb_height / 4;

static GuiNumber loco_num(fb, loco_col, loco_row, screen_bg,
                          roboto_48_digit_img, 3, HAlign::Center);

///// Function Buttons
// Horn Lights
// Bell Engine

static constexpr Font f_btn_font = roboto_28;
static constexpr int btn_brd = 2;
static constexpr int f_btn_wid = 100;
static constexpr int f_btn_hgt = 50;
static constexpr int f_btn_col_1 = 10;
static constexpr int f_btn_col_2 = fb_width - f_btn_col_1 - f_btn_wid;
static constexpr int f_btn_spc = 20;
static constexpr int f_btn_row_1 = 60;
static constexpr int f_btn_row_2 = f_btn_row_1 + f_btn_hgt + f_btn_spc;

static void horn_btn_dn(intptr_t);
static void horn_btn_up(intptr_t);

BUTTON_1(horn, "Horn", fb, f_btn_col_1, f_btn_row_1, f_btn_wid, f_btn_hgt,
         btn_brd, f_btn_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,     // on_click
         horn_btn_dn, 0, // on_down
         horn_btn_up, 0, // on_up
         GuiButton::Mode::Momentary, false);

static void horn_btn_dn(intptr_t)
{
    if (throttle == nullptr)
        return;
    throttle->set_function(2, true); // F2 on
}

static void horn_btn_up(intptr_t)
{
    if (throttle == nullptr)
        return;
    throttle->set_function(2, false); // F2 off
}

static void bell_btn_dn(intptr_t);

BUTTON_1(bell, "Bell", fb, f_btn_col_1, f_btn_row_2, f_btn_wid, f_btn_hgt,
         btn_brd, f_btn_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,     // on_click
         bell_btn_dn, 0, // on_down
         nullptr, 0,     // on_up
         GuiButton::Mode::Check, false);

static void bell_btn_dn(intptr_t)
{
    if (throttle == nullptr)
        return;
    throttle->set_function(1, bell_btn.pressed()); // F1
}

static void lights_btn_dn(intptr_t);

BUTTON_1(lights, "Lights", fb, f_btn_col_2, f_btn_row_1, f_btn_wid, f_btn_hgt,
         btn_brd, f_btn_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,       // on_click
         lights_btn_dn, 0, // on_down
         nullptr, 0,       // on_up
         GuiButton::Mode::Check, false);

static void lights_btn_dn(intptr_t)
{
    if (throttle == nullptr)
        return;
    throttle->set_function(0, lights_btn.pressed()); // F0
}

static void engine_btn_dn(intptr_t);

BUTTON_1(engine, "Engine", fb, f_btn_col_2, f_btn_row_2, f_btn_wid, f_btn_hgt,
         btn_brd, f_btn_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,       // on_click
         engine_btn_dn, 0, // on_down
         nullptr, 0,       // on_up
         GuiButton::Mode::Check, false);

static void engine_btn_dn(intptr_t)
{
    if (throttle == nullptr)
        return;
    throttle->set_function(8, engine_btn.pressed()); // F8
}

///// Speed
// req_num  rev_btn  stop_btn  fwd_btn  act_num
//                  speed_sld

// Layout

static constexpr int speed_mrg = 10; // sides
static constexpr int speed_bot = 2;  // bottom
static constexpr int speed_wid = fb_width - 2 * speed_mrg;
static constexpr int speed_hgt = 40;
static constexpr int speed_row = fb_height - speed_bot - speed_hgt;

static constexpr int req_col = speed_mrg;
static constexpr int act_col = fb_width - speed_mrg;
static constexpr int spd_spc = 30;
static constexpr int spd_hgt = 36; // roboto_36
static constexpr int req_row = speed_row - spd_spc - spd_hgt;

static constexpr int dir_wid = 100;
static constexpr int dir_hgt = 50;
static constexpr int dir_wid2 = dir_wid / 2;
static constexpr int dir_spc = 0;
static constexpr int dir_row = req_row;
static constexpr Font dir_font = roboto_28;
static constexpr int rev_col = fb_width2 - dir_wid2 - dir_spc - dir_wid;
static constexpr int stop_col = fb_width2 - dir_wid2;
static constexpr int fwd_col = fb_width2 + dir_wid2 + dir_spc;

// Speed Requested and Actual

static GuiNumber req_num(fb, req_col, req_row, screen_bg, roboto_36_digit_img,
                         0, HAlign::Left);

static GuiNumber act_num(fb, act_col, req_row, screen_bg, roboto_36_digit_img,
                         0, HAlign::Right);

// Speed Slider

static void speed_change(intptr_t);

static GuiSlider speed_sld(fb,                   //
                           speed_mrg, speed_row, // col, row
                           speed_wid, speed_hgt, // wid, hgt
                           screen_fg, screen_bg, // fg, bg
                           Color::gray(90),      // track_bg
                           Color::white(),       // handle_bg
                           0, 127, 0,            // min, max, init
                           speed_change, 0);     // on_value()

// Forward Button

static void fwd_dn(intptr_t);

BUTTON_1(fwd, "Forward", fb, fwd_col, dir_row, dir_wid, dir_hgt, btn_brd,
         dir_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,                                           // on_click
         fwd_dn, 0,                                            // on_down
         nullptr, 0,                                           // on_up
         GuiButton::Mode::Radio, false);

// Stop Button

static void stop_dn(intptr_t);

BUTTON_1(stop, "Stop", fb, stop_col, dir_row, dir_wid, dir_hgt, btn_brd,
         dir_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,                                           // on_click
         stop_dn, 0,                                           // on_down
         nullptr, 0,                                           // on_up
         GuiButton::Mode::Radio, true);

// Reverse Button

static void rev_dn(intptr_t);

BUTTON_1(rev, "Reverse", fb, rev_col, dir_row, dir_wid, dir_hgt, btn_brd,
         dir_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,                                           // on_click
         rev_dn, 0,                                            // on_down
         nullptr, 0,                                           // on_up
         GuiButton::Mode::Radio, false);

static void set_speed()
{
    if (throttle == nullptr)
        return;

    int speed = speed_sld.get_value();
    assert(0 <= speed && speed <= 127);

    if (fwd_btn.pressed()) {
        throttle->set_speed(speed);
    } else if (rev_btn.pressed()) {
        throttle->set_speed(-speed);
    } else {
        throttle->set_speed(0);
    }
}

static void speed_change(intptr_t)
{
    int speed = speed_sld.get_value();
    req_num.set_value(speed);
    set_speed();
}

static void fwd_dn(intptr_t)
{
    stop_btn.pressed(false);
    rev_btn.pressed(false);
    set_speed();
}

static void stop_dn(intptr_t)
{
    fwd_btn.pressed(false);
    rev_btn.pressed(false);
    set_speed();
}

static void rev_dn(intptr_t)
{
    fwd_btn.pressed(false);
    stop_btn.pressed(false);
    set_speed();
}

static void update(intptr_t)
{
    // see if railcom-reported speed has changed
    static int rc_speed_last = INT_MAX;
    int rc_speed = throttle->get_rc_speed();
    if (rc_speed != rc_speed_last) {
        act_num.set_value(rc_speed);
        rc_speed_last = rc_speed;
    }
}

///// Main Page

// clang-format off
static GuiPage page({
    &horn_btn,          &loco_num,           &lights_btn,
    &bell_btn,                               &engine_btn,
    &req_num, &rev_btn, &stop_btn, &fwd_btn, &act_num,
                        &speed_sld
}, update, 0);
// clang-format on

} // namespace MainPage

//////////////////////////////////////////////////////////////////////////////
///// LOCO ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

namespace LocoPage {

static GuiPage page({});

} // namespace LocoPage

//////////////////////////////////////////////////////////////////////////////
///// FUNC ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

namespace FuncPage {

static GuiPage page({});

} // namespace FuncPage

//////////////////////////////////////////////////////////////////////////////
///// PROG ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

namespace ProgPage {

// +------------------------------------+
// | [MAIN] [LOCO] [FUNC] [PROG] [MORE] |
// |                                    |
// | [CV Num] nnnn    [ 7 ] [ 8 ] [ 9 ] |
// |                                    |
// | [CV Val]  vvv    [ 4 ] [ 5 ] [ 6 ] |
// |                                    |
// |     status       [ 1 ] [ 2 ] [ 3 ] |
// |                                    |
// | [Read] [Write]   [BS ] [ 0 ] [CLR] |
// +------------------------------------+

static constexpr int btn_brd = 2;

static constexpr int btn_wid = 140;
static constexpr int btn_hgt = 50;
static constexpr int btn_col = 20;
static constexpr int btn_row_1 = 60 + 7;
static constexpr int btn_row_2 = btn_row_1 + 65;

static constexpr Font btn_font = roboto_36;
static const PixelImageHdr **btn_digit_img = roboto_36_digit_img;

static constexpr int val_wid = 80;
static constexpr int val_col = btn_col + btn_wid + 10 + val_wid;
static constexpr int val_row_1 = btn_row_1 + (btn_hgt - btn_font.y_adv) / 2;
static constexpr int val_row_2 = btn_row_2 + (btn_hgt - btn_font.y_adv) / 2;

// CV Num and CV Val Buttons and Values

static void cv_num_btn_dn(intptr_t);

BUTTON_1(cv_num, "CV Num", fb, btn_col, btn_row_1, btn_wid, btn_hgt, btn_brd,
         btn_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,                                           // on_click
         cv_num_btn_dn, 0,                                     // on_down
         nullptr, 0,                                           // on_up
         GuiButton::Mode::Radio, true);

static GuiNumber cv_num_val(fb, val_col, val_row_1, screen_bg, btn_digit_img,
                            GuiNumber::unset, HAlign::Right);

static void cv_val_btn_dn(intptr_t);

BUTTON_1(cv_val, "CV Val", fb, btn_col, btn_row_2, btn_wid, btn_hgt, btn_brd,
         btn_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         nullptr, 0,                                           // on_click
         cv_val_btn_dn, 0,                                     // on_down
         nullptr, 0,                                           // on_up
         GuiButton::Mode::Radio, false);

static GuiNumber cv_val_val(fb, val_col, val_row_2, screen_bg, btn_digit_img,
                            GuiNumber::unset, HAlign::Right);

// Read and Write Buttons

static constexpr int rw_btn_wid = 100;
static constexpr int rw_btn_hgt = 50;

static constexpr int rw_btn_spc = 20;

static constexpr int rd_btn_col = 20;
static constexpr int rd_btn_row = fb_height - 20 - rw_btn_hgt;

static constexpr int wr_btn_col = rd_btn_col + rw_btn_wid + rw_btn_spc;
static constexpr int wr_btn_row = rd_btn_row;

static void rd_btn_click(intptr_t);

BUTTON_1(rd, "Read", fb, rd_btn_col, rd_btn_row, rw_btn_wid, rw_btn_hgt,
         btn_brd, btn_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         rd_btn_click, 0, // on_click
         nullptr, 0,      // on_down
         nullptr, 0,      // on_up
         GuiButton::Mode::Momentary, false);

static void wr_btn_click(intptr_t);

BUTTON_1(wr, "Write", fb, wr_btn_col, wr_btn_row, rw_btn_wid, rw_btn_hgt,
         btn_brd, btn_font, screen_fg, screen_bg, btn_up_bg, btn_dn_bg, //
         wr_btn_click, 0, // on_click
         nullptr, 0,      // on_down
         nullptr, 0,      // on_up
         GuiButton::Mode::Momentary, false);

// status message (none or one will be visible)

static constexpr int stat_ctr = rd_btn_col + rw_btn_wid + rw_btn_spc / 2;
static constexpr int stat_wid = 200;
static constexpr int stat_hgt = 40;
static constexpr int stat_col = stat_ctr - stat_wid / 2;
static constexpr int stat_row = rd_btn_row - stat_hgt - 10;
static constexpr Font stat_font = roboto_28;

static constexpr PixelImage<Pixel565, stat_wid, stat_hgt> work_img =
    label_img<Pixel565, stat_wid, stat_hgt> //
    ("Working", stat_font, screen_fg, 0, screen_fg, screen_bg);

static GuiLabel work_lbl(fb, stat_col, stat_row, screen_bg,    //
                         &work_img.hdr, &work_img.hdr, false); // not visible

static constexpr PixelImage<Pixel565, stat_wid, stat_hgt> ok_img =
    label_img<Pixel565, stat_wid, stat_hgt> //
    ("OK", stat_font, screen_fg, 0, screen_fg, screen_bg);

static GuiLabel ok_lbl(fb, stat_col, stat_row, screen_bg, //
                       &ok_img.hdr, &ok_img.hdr, false);  // not visible

static constexpr PixelImage<Pixel565, stat_wid, stat_hgt> err_img =
    label_img<Pixel565, stat_wid, stat_hgt> //
    ("Error", stat_font, screen_fg, 0, screen_fg, screen_bg);

static GuiLabel err_lbl(fb, stat_col, stat_row, screen_bg,  //
                        &err_img.hdr, &err_img.hdr, false); // not visible

static GuiLabel *cur_stat = nullptr;

static void set_status(GuiLabel *stat)
{
    if (stat == cur_stat)
        return;

    if (cur_stat != nullptr) {
        cur_stat->erase();
        cur_stat->visible(false);
    }

    cur_stat = stat;

    if (cur_stat != nullptr) {
        cur_stat->visible(true);
        cur_stat->draw();
    }
}

// update page
static void update(intptr_t);

// clang-format off
static GuiPage page({
    &cv_num_btn, &cv_num_val, &NumPad::pad_7_btn,  &NumPad::pad_8_btn, &NumPad::pad_9_btn,   //
    &cv_val_btn, &cv_val_val, &NumPad::pad_4_btn,  &NumPad::pad_5_btn, &NumPad::pad_6_btn,   //
         &ok_lbl,             &NumPad::pad_1_btn,  &NumPad::pad_2_btn, &NumPad::pad_3_btn,   //
    &rd_btn, &wr_btn,         &NumPad::pad_bs_btn, &NumPad::pad_0_btn, &NumPad::pad_clr_btn, //
    // these go where ok_lbl is
    &work_lbl, &err_lbl,
}, update, 0);
// clang-format on

} // namespace ProgPage

//////////////////////////////////////////////////////////////////////////////
///// MORE ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

namespace MorePage {

static GuiPage page({});

} // namespace MorePage

//////////////////////////////////////////////////////////////////////////////
///// All Pages //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

static int active_page = -1;

static GuiPage *pages[] = {
    &MainPage::page, &LocoPage::page, &FuncPage::page,
    &ProgPage::page, &MorePage::page,
};

static constexpr int pages_cnt = sizeof(pages) / sizeof(pages[0]);

//////////////////////////////////////////////////////////////////////////////
///// Navigation Buttons /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

namespace Nav {

static constexpr Font font = roboto_28;

// Colors here are "backwards" from usual. The button for the current page
// is the same as the page background (white) and disabled, while the other
// buttons are shadowed (gray) and are enabled to allow navigation to their
// pages.
static constexpr Color bg_ena = Color::gray(80); // buttons to other pages
static constexpr Color bg_dis = screen_bg;       // button for current page
static constexpr Color bg_prs = Color::gray(50);

static constexpr int brd_thk_ena = 4;
static constexpr int brd_thk_dis = 1;
static constexpr int brd_thk_prs = 6;
static constexpr int brd_thk_max = 6;

static constexpr int btn_hgt = font.y_adv + 2 * brd_thk_max; // 40
static constexpr int btn_wid = fb_width / pages_cnt;         // 96

static void nav_click(int page_num);

// clang-format off
#define NAV_BUTTON(N, TXT) \
    static constexpr PixelImage<Pixel565, btn_wid, btn_hgt> b##N##_ena_img = \
        label_img<Pixel565, btn_wid, btn_hgt> \
            (TXT, font, screen_fg, brd_thk_ena, screen_fg, bg_ena); \
    \
    static constexpr PixelImage<Pixel565, btn_wid, btn_hgt> b##N##_dis_img = \
        label_img<Pixel565, btn_wid, btn_hgt> \
            (TXT, font, screen_fg, brd_thk_dis, screen_fg, bg_dis); \
    \
    static constexpr PixelImage<Pixel565, btn_wid, btn_hgt> b##N##_prs_img = \
        label_img<Pixel565, btn_wid, btn_hgt> \
            (TXT, font, screen_fg, brd_thk_prs, screen_fg, bg_prs); \
    \
    static GuiButton nav_##N(fb, N * fb_width / pages_cnt, 0, screen_bg, \
                             &b##N##_ena_img.hdr, \
                             &b##N##_dis_img.hdr, \
                             &b##N##_prs_img.hdr, \
                             nav_click, N, nullptr, 0, nullptr, 0);
// clang-format on

NAV_BUTTON(0, "MAIN")
NAV_BUTTON(1, "LOCO")
NAV_BUTTON(2, "FUNC")
NAV_BUTTON(3, "PROG")
NAV_BUTTON(4, "MORE")

#undef NAV_BUTTON

static GuiButton *btns[] = {&nav_0, &nav_1, &nav_2, &nav_3, &nav_4};

static void show_page(int page_num)
{
    btns[page_num]->enabled(false);
    pages[page_num]->visible(true);
}

static void hide_page(int page_num, bool force_draw = false)
{
    btns[page_num]->enabled(true, force_draw);
    pages[page_num]->visible(false);
}

static void nav_click(int page_num)
{
    assert(0 <= page_num && page_num < pages_cnt);

    if (active_page == -1) {
        // first call only, hide them all and force redraw
        for (int p = 0; p < pages_cnt; p++)
            hide_page(p, true);
        active_page = page_num;
        show_page(active_page);
        return;
    }

    if (pages[active_page]->busy() != 0)
        return; // ignore navigation while page is busy

    // switch pages
    hide_page(active_page);
    active_page = page_num;
    show_page(active_page);
}

} // namespace Nav

static void ProgPage::update(intptr_t)
{
    int busy = pages[active_page]->busy();
    if (busy == 0) {
        // not busy
        return;
    } else {
        assert(busy == 1 || busy == 2);
        // reading or writing
        bool result;
        uint8_t value;
        if (throttle->ops_done(result, value)) {
            if (result) {
                cv_val_val.set_value(value);
                set_status(&ok_lbl);
            } else {
                set_status(&err_lbl);
            }
            pages[active_page]->busy(0); // not busy
        }
    }
}

static void ProgPage::cv_num_btn_dn(intptr_t)
{
    cv_num_val.set_value(GuiNumber::unset);
    cv_val_btn.pressed(false);
    set_status(nullptr); // clear status
}

static void ProgPage::cv_val_btn_dn(intptr_t)
{
    cv_val_val.set_value(GuiNumber::unset);
    cv_num_btn.pressed(false);
    set_status(nullptr); // clear status
}

static void ProgPage::rd_btn_click(intptr_t)
{
    int cv_num = cv_num_val.get_value();
    if (cv_num == GuiNumber::unset)
        return;
    throttle->read_cv(cv_num);
    pages[active_page]->busy(1);
    cv_val_val.set_value(GuiNumber::unset);
    set_status(&work_lbl);
}

static void ProgPage::wr_btn_click(intptr_t)
{
    int cv_num = cv_num_val.get_value();
    int cv_val = cv_val_val.get_value();
    if (cv_num == GuiNumber::unset || cv_val == GuiNumber::unset)
        return;
    throttle->write_cv(cv_num, cv_val);
    pages[active_page]->busy(2);
    cv_val_val.set_value(GuiNumber::unset);
    set_status(&work_lbl);
}

// update number helper
static void NumPad::update_num(GuiNumber &num, int val, int min_val,
                               int max_val)
{
    int cur_val = num.get_value();
    if (cur_val == GuiNumber::unset) {
        if (val >= min_val && val <= max_val)
            num.set_value(val);
    } else if (val == btn_bs_val) {
        // backspace
        int new_val = cur_val / 10;
        if (new_val < min_val)
            num.set_value(GuiNumber::unset);
        else
            num.set_value(new_val);
    } else if (val == btn_clr_val) {
        // clear
        num.set_value(GuiNumber::unset);
    } else {
        // digit
        assert(0 <= val && val <= 9);
        int new_val = cur_val * 10 + val;
        if (new_val >= min_val && new_val <= max_val)
            num.set_value(new_val);
        // else ignore keypress
    }
}

static void NumPad::btn_dn(intptr_t n)
{
    if (active_page == 0) {
        // main page
        ;
    } else if (active_page == 1) {
        // loco page
        ;
    } else if (active_page == 2) {
        // func page
        ;
    } else if (active_page == 3) {
        // prog page

        if (ProgPage::cv_num_btn.pressed())
            update_num(ProgPage::cv_num_val, n, 1, 1024);
        else
            update_num(ProgPage::cv_val_val, n, 0, 255);

        ProgPage::set_status(nullptr); // clear status

    } else {
        assert(active_page == 4);
        // more page
        ;
    }
}


//////////////////////////////////////////////////////////////////////////////
///// Main ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

int main()
{
    stdio_init_all();
    SysLed::init();

    SysLed::pattern(50, 950);

#if 0
    while (!stdio_usb_connected()) {
        SysLed::loop();
        tight_loop_contents();
    }
    sleep_ms(10); // small delay needed or we lose the first prints
#endif

    SysLed::off();

    printf("\n");
    printf("throttle\n");
    printf("\n");

    Argv argv(1); // verbosity == 1 means echo

    // initialize framebuffer
    fb_spi_baud_actual = fb.spi_freq();
    fb.init();
    fb.set_rotation(Framebuffer::Rotation::landscape); // connector to the left
    fb.fill_rect(0, 0, fb.width(), fb.height(), screen_bg);
    printf("framebuffer ready");
    //printf(" (spi @ %d Hz)\n", fb_spi_baud_actual);
    printf("\n");

    Nav::nav_click(0); // start out on page 0

    fb.wait_idle();
    fb.brightness(100);

    // initialize touchscreen
    ts_i2c_baud_actual = i2c_dev.baud();
    assert(ts.init());
    ts.set_rotation(Touchscreen::Rotation::landscape);
    printf("touchscreen ready");
    //printf(" (i2c @ %u Hz)\n", ts_i2c_baud_actual);
    printf("\n");

    throttle = command.create_throttle(); // default address 3

    command.set_mode_ops(); // track power on

    while (true) {

        Event event(ts.get_event());
        if (event.type != Event::Type::none) {
            // anyone have focus?
            if (GuiWidget::focus != nullptr) {
                // yes, send event there
                GuiWidget::focus->event(event);
            } else {
                // no, see if anyone wants it
                bool handled = false;
                // nav buttons?
                for (int p = 0; p < pages_cnt && !handled; p++)
                    handled = Nav::btns[p]->event(event);
                // anyone on current page want it?
                if (!handled)
                    pages[active_page]->event(event);
            }
        }

        // let active page update itself if it wants to
        pages[active_page]->update();

    } // while (true)

    sleep_ms(100);

    return 0;

} // int main()
