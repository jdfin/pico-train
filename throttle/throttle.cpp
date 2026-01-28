
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
#include "st7796.h"
// touchscreen
#include "gt911.h"
// gui
#include "gui_button.h"
#include "gui_label.h"
#include "gui_macros.h"
#include "gui_number.h"
#include "gui_page.h"
#include "gui_slider.h"
//
#include "fb_gpio_cfg.h"
#include "ts_gpio_cfg.h"

using HAlign = Framebuffer::HAlign;

using Event = Touchscreen::Event;

static const int fb_spi_baud_request = 15'000'000;
static int fb_spi_baud_actual = 0;

static const uint ts_i2c_baud_request = 400'000;
static uint ts_i2c_baud_actual = 0;
static const uint8_t ts_i2c_addr = 0x14; // 0x14 or 0x5d

static const int work_bytes = 128;
static uint8_t work[work_bytes];

static St7796 fb(fb_spi_inst, fb_spi_miso_gpio, fb_spi_mosi_gpio, fb_spi_clk_gpio, fb_spi_cs_gpio,
                 fb_spi_baud_request, fb_cd_gpio, fb_rst_gpio, fb_led_gpio, 480, 320,
                 work, work_bytes);

static I2cDev i2c_dev(ts_i2c_inst, ts_i2c_scl_gpio, ts_i2c_sda_gpio, ts_i2c_baud_request);

static Gt911 ts(i2c_dev, ts_i2c_addr, ts_rst_gpio, ts_int_gpio);

static constexpr Color screen_bg = Color::white();
static constexpr Color screen_fg = Color::black();

static constexpr int page_cnt = 5;

// fb.width() is not constexpr
static constexpr int fb_width = 480;
static constexpr int fb_height = 320;

static constexpr int fb_width2 = fb_width / 2;

DIGIT_IMAGE_ARRAY(roboto_36, screen_fg, screen_bg);
DIGIT_IMAGE_ARRAY(roboto_48, screen_fg, screen_bg);

static constexpr Color btn_up_bg = Color::gray(95);
static constexpr Color btn_dn_bg = Color::gray(85);

//////////////////////////////////////////////////////////////////////////////
///// Main Page //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

///// Loco Number

static constexpr int p1_loco_col = fb_width / 2;
static constexpr int p1_loco_row = fb_height / 4;
static GuiNumber p1_loco_num(fb, p1_loco_col, p1_loco_row, screen_bg,
                             roboto_48_digit_img, 7956, HAlign::Center);

///// Function Buttons

static constexpr Font p1_func_font = roboto_28;
static constexpr int p1_brd = 2;
static constexpr int p1_func_wid = 100;
static constexpr int p1_func_hgt = 50;
static constexpr int p1_func_mrg = 10;
static constexpr int p1_func_spc = 20;
static constexpr int p1_func_row_1 = 60;
static constexpr int p1_func_row_2 = p1_func_row_1 + p1_func_hgt + p1_func_spc;

// Horn Button

static constexpr PixelImage<Pixel565, p1_func_wid, p1_func_hgt> p1_horn_img_up =
    label_img<Pixel565, p1_func_wid, p1_func_hgt> //
    ("Horn", p1_func_font, screen_fg, p1_brd, screen_fg, btn_up_bg);

static constexpr PixelImage<Pixel565, p1_func_wid, p1_func_hgt> p1_horn_img_dn =
    label_img<Pixel565, p1_func_wid, p1_func_hgt> //
    ("Horn", p1_func_font, screen_fg, p1_brd, screen_fg, btn_dn_bg);
static void p1_horn_dn(intptr_t);
static void p1_horn_up(intptr_t);

static GuiButton p1_horn_btn(fb, p1_func_mrg, p1_func_row_1, screen_bg,
                             &p1_horn_img_up.hdr, &p1_horn_img_up.hdr,
                             &p1_horn_img_dn.hdr, nullptr, 0, p1_horn_dn, 0,
                             p1_horn_up, 0);

static void p1_horn_dn(intptr_t)
{
    printf("Horn ON\n");
}

static void p1_horn_up(intptr_t)
{
    printf("Horn OFF\n");
}

// Bell Button

static constexpr PixelImage<Pixel565, p1_func_wid, p1_func_hgt> p1_bell_img_up =
    label_img<Pixel565, p1_func_wid, p1_func_hgt> //
    ("Bell", p1_func_font, screen_fg, p1_brd, screen_fg, btn_up_bg);

static constexpr PixelImage<Pixel565, p1_func_wid, p1_func_hgt> p1_bell_img_dn =
    label_img<Pixel565, p1_func_wid, p1_func_hgt> //
    ("Bell", p1_func_font, screen_fg, p1_brd, screen_fg, btn_dn_bg);

static void p1_bell_dn(intptr_t);
static GuiButton p1_bell_btn(fb, p1_func_mrg, p1_func_row_2, screen_bg,
                             &p1_bell_img_up.hdr, &p1_bell_img_up.hdr,
                             &p1_bell_img_dn.hdr, nullptr, 0, p1_bell_dn, 0,
                             nullptr, 0, GuiButton::Mode::Check);

static void p1_bell_dn(intptr_t)
{
    if (p1_bell_btn.pressed())
        printf("Bell ON\n");
    else
        printf("Bell OFF\n");
}

// Lights Button

static constexpr PixelImage<Pixel565, p1_func_wid, p1_func_hgt>
    p1_lights_img_up = label_img<Pixel565, p1_func_wid, p1_func_hgt> //
    ("Lights", p1_func_font, screen_fg, p1_brd, screen_fg, btn_up_bg);

static constexpr PixelImage<Pixel565, p1_func_wid, p1_func_hgt>
    p1_lights_img_dn = label_img<Pixel565, p1_func_wid, p1_func_hgt> //
    ("Lights", p1_func_font, screen_fg, p1_brd, screen_fg, btn_dn_bg);

static void p1_lights_dn(intptr_t);

static GuiButton p1_lights_btn(fb, fb_width - p1_func_mrg - p1_func_wid,
                               p1_func_row_1, screen_bg, &p1_lights_img_up.hdr,
                               &p1_lights_img_up.hdr, &p1_lights_img_dn.hdr,
                               nullptr, 0, p1_lights_dn, 0, nullptr, 0,
                               GuiButton::Mode::Check);

static void p1_lights_dn(intptr_t)
{
    if (p1_lights_btn.pressed())
        printf("Lights ON\n");
    else
        printf("Lights OFF\n");
}

// Engine Button

static constexpr PixelImage<Pixel565, p1_func_wid, p1_func_hgt>
    p1_engine_img_up = label_img<Pixel565, p1_func_wid, p1_func_hgt> //
    ("Engine", p1_func_font, screen_fg, p1_brd, screen_fg, btn_up_bg);

static constexpr PixelImage<Pixel565, p1_func_wid, p1_func_hgt>
    p1_engine_img_dn = label_img<Pixel565, p1_func_wid, p1_func_hgt> //
    ("Engine", p1_func_font, screen_fg, p1_brd, screen_fg, btn_dn_bg);

static void p1_engine_dn(intptr_t);

static GuiButton p1_engine_btn(fb, fb_width - p1_func_mrg - p1_func_wid,
                               p1_func_row_2, screen_bg, &p1_engine_img_up.hdr,
                               &p1_engine_img_up.hdr, &p1_engine_img_dn.hdr,
                               nullptr, 0, p1_engine_dn, 0, nullptr, 0,
                               GuiButton::Mode::Check);

static void p1_engine_dn(intptr_t)
{
    if (p1_engine_btn.pressed())
        printf("Engine ON\n");
    else
        printf("Engine OFF\n");
}

///// Speed

// Layout

static constexpr int p1_speed_mrg = 10; // sides
static constexpr int p1_speed_bot = 2; // bottom
static constexpr int p1_speed_wid = fb_width - 2 * p1_speed_mrg;
static constexpr int p1_speed_hgt = 40;
static constexpr int p1_speed_row = fb_height - p1_speed_bot - p1_speed_hgt;

static constexpr int p1_req_col = p1_speed_mrg;
static constexpr int p1_act_col = fb_width - p1_speed_mrg;
static constexpr int p1_spd_spc = 30;
static constexpr int p1_spd_hgt = 36; // roboto_36
static constexpr int p1_req_row = p1_speed_row - p1_spd_spc - p1_spd_hgt;

static constexpr int p1_dir_wid = 100;
static constexpr int p1_dir_hgt = 50;
static constexpr int p1_dir_wid2 = p1_dir_wid / 2;
static constexpr int p1_dir_spc = 0;
static constexpr int p1_dir_row = p1_req_row;
static constexpr Font p1_dir_font = roboto_28;
static constexpr int p1_rev_col = fb_width2 - p1_dir_wid2 - p1_dir_spc - p1_dir_wid;
static constexpr int p1_stop_col = fb_width2 - p1_dir_wid2;
static constexpr int p1_fwd_col = fb_width2 + p1_dir_wid2 + p1_dir_spc;

// Speed Requested and Actual

static GuiNumber p1_req_num(fb, p1_req_col, p1_req_row, screen_bg,
                            roboto_36_digit_img, 0, HAlign::Left);

static GuiNumber p1_act_num(fb, p1_act_col, p1_req_row, screen_bg,
                            roboto_36_digit_img, 0, HAlign::Right);

// Speed Slider

static void p1_speed_change(intptr_t);

static GuiSlider p1_speed_sld(fb,                         //
                              p1_speed_mrg, p1_speed_row, // col, row
                              p1_speed_wid, p1_speed_hgt, // wid, hgt
                              screen_fg, screen_bg,       // fg, bg
                              Color::gray(90),            // track_bg
                              Color::white(),             // handle_bg
                              0, 127, 0,                  // min, max, init
                              p1_speed_change, 0);        // on_value()

static void p1_speed_change(intptr_t)
{
    int speed = p1_speed_sld.get_value();
    printf("Speed: %d\n", speed);
    p1_req_num.set_value(speed);
}

// Forward Button

static constexpr PixelImage<Pixel565, p1_dir_wid, p1_dir_hgt> p1_fwd_img_up =
    label_img<Pixel565, p1_dir_wid, p1_dir_hgt> //
    ("Forward", p1_dir_font, screen_fg, p1_brd, screen_fg, btn_up_bg);

static constexpr PixelImage<Pixel565, p1_dir_wid, p1_dir_hgt> p1_fwd_img_dn =
    label_img<Pixel565, p1_dir_wid, p1_dir_hgt> //
    ("Forward", p1_dir_font, screen_fg, p1_brd, screen_fg, btn_dn_bg);

static void p1_fwd_dn(intptr_t);

static GuiButton p1_fwd_btn(fb, p1_fwd_col, p1_dir_row, screen_bg,
                            &p1_fwd_img_up.hdr, &p1_fwd_img_up.hdr,
                            &p1_fwd_img_dn.hdr, nullptr, 0, p1_fwd_dn, 0,
                            nullptr, 0, GuiButton::Mode::Radio, false);

// Stop Button

static constexpr PixelImage<Pixel565, p1_dir_wid, p1_dir_hgt> p1_stop_img_up =
    label_img<Pixel565, p1_dir_wid, p1_dir_hgt> //
    ("Stop", p1_dir_font, screen_fg, p1_brd, screen_fg, btn_up_bg);

static constexpr PixelImage<Pixel565, p1_dir_wid, p1_dir_hgt> p1_stop_img_dn =
    label_img<Pixel565, p1_dir_wid, p1_dir_hgt> //
    ("Stop", p1_dir_font, screen_fg, p1_brd, screen_fg, btn_dn_bg);

static void p1_stop_dn(intptr_t);

static GuiButton p1_stop_btn(fb, p1_stop_col, p1_dir_row, screen_bg,
                             &p1_stop_img_up.hdr, &p1_stop_img_up.hdr,
                             &p1_stop_img_dn.hdr, nullptr, 0, p1_stop_dn, 0,
                             nullptr, 0, GuiButton::Mode::Radio, true);

// Reverse Button

static constexpr PixelImage<Pixel565, p1_dir_wid, p1_dir_hgt> p1_rev_img_up =
    label_img<Pixel565, p1_dir_wid, p1_dir_hgt> //
    ("Reverse", p1_dir_font, screen_fg, p1_brd, screen_fg, btn_up_bg);

static constexpr PixelImage<Pixel565, p1_dir_wid, p1_dir_hgt> p1_rev_img_dn =
    label_img<Pixel565, p1_dir_wid, p1_dir_hgt> //
    ("Reverse", p1_dir_font, screen_fg, p1_brd, screen_fg, btn_dn_bg);

static void p1_rev_dn(intptr_t);

static GuiButton p1_rev_btn(fb, p1_rev_col, p1_dir_row, screen_bg,
                            &p1_rev_img_up.hdr, &p1_rev_img_up.hdr,
                            &p1_rev_img_dn.hdr, nullptr, 0, p1_rev_dn, 0,
                            nullptr, 0, GuiButton::Mode::Radio, false);

static void p1_fwd_dn(intptr_t)
{
    printf("Forward\n");
    p1_stop_btn.pressed(false);
    p1_rev_btn.pressed(false);
}

static void p1_stop_dn(intptr_t)
{
    printf("Stop\n");
    p1_fwd_btn.pressed(false);
    p1_rev_btn.pressed(false);
}

static void p1_rev_dn(intptr_t)
{
    printf("Reverse\n");
    p1_fwd_btn.pressed(false);
    p1_stop_btn.pressed(false);
}

///// Main Page

// clang-format off
static GuiPage main_page({
    &p1_horn_btn,             &p1_loco_num,           &p1_lights_btn,
    &p1_bell_btn,                                     &p1_engine_btn,
    &p1_req_num, &p1_rev_btn, &p1_stop_btn, &p1_fwd_btn, &p1_act_num,
                              &p1_speed_sld
});
// clang-format on

//////////////////////////////////////////////////////////////////////////////
///// Loco Page //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

static GuiPage loco_page({});

//////////////////////////////////////////////////////////////////////////////
///// Func Page //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

static GuiPage func_page({});

//////////////////////////////////////////////////////////////////////////////
///// Prog Page //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

static GuiPage prog_page({});

//////////////////////////////////////////////////////////////////////////////
///// More Page //////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

static GuiPage more_page({});

//////////////////////////////////////////////////////////////////////////////
///// Navigation Buttons /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

static int active_page = -1;

static GuiPage *pages[] = {
    &main_page, &loco_page, &func_page, &prog_page, &more_page,
};

static constexpr int nav_cnt = page_cnt;
static constexpr Font nav_font = roboto_28;

static constexpr Color nav_bg_ena = Color::gray(80);
static constexpr Color nav_bg_dis = screen_bg;
static constexpr Color nav_bg_prs = Color::gray(50);

static constexpr int nav_brd_thk_ena = 4;
static constexpr int nav_brd_thk_dis = 1;
static constexpr int nav_brd_thk_prs = 6;
static constexpr int nav_brd_thk_max = 6;

static constexpr int nav_hgt = nav_font.y_adv + 2 * nav_brd_thk_max; // 40
static constexpr int nav_wid = fb_width / nav_cnt;                   // 96

static void nav_click(int page_num);

// clang-format off
#define NAV_BUTTON(N, TXT) \
    static constexpr PixelImage<Pixel565, nav_wid, nav_hgt> b##N##_img_ena = \
        label_img<Pixel565, nav_wid, nav_hgt>(TXT, nav_font, screen_fg, \
                                              nav_brd_thk_ena, screen_fg, \
                                              nav_bg_ena); \
    \
    static constexpr PixelImage<Pixel565, nav_wid, nav_hgt> b##N##_img_dis = \
        label_img<Pixel565, nav_wid, nav_hgt>(TXT, nav_font, screen_fg, \
                                              nav_brd_thk_dis, screen_fg, \
                                              nav_bg_dis); \
    \
    static constexpr PixelImage<Pixel565, nav_wid, nav_hgt> b##N##_img_prs = \
        label_img<Pixel565, nav_wid, nav_hgt>(TXT, nav_font, screen_fg, \
                                              nav_brd_thk_prs, screen_fg, \
                                              nav_bg_prs); \
    \
    static GuiButton nav_##N(fb, N * fb_width / nav_cnt, 0, screen_bg, \
                             &b##N##_img_ena.hdr, \
                             &b##N##_img_dis.hdr, \
                             &b##N##_img_prs.hdr, \
                             nav_click, N, nullptr, 0, nullptr, 0);
// clang-format on

NAV_BUTTON(0, "MAIN")
NAV_BUTTON(1, "LOCO")
NAV_BUTTON(2, "FUNC")
NAV_BUTTON(3, "PROG")
NAV_BUTTON(4, "MORE")

#undef NAV_BUTTON

static GuiButton *navs[] = {&nav_0, &nav_1, &nav_2, &nav_3, &nav_4};

static void show_page(int page_num)
{
    navs[page_num]->enabled(false);
    pages[page_num]->visible(true);
}

static void hide_page(int page_num, bool force_draw = false)
{
    navs[page_num]->enabled(true, force_draw);
    pages[page_num]->visible(false);
}

static void nav_click(int page_num)
{
    assert(0 <= page_num && page_num < page_cnt);

    if (active_page == -1) {
        // first call only, hide them all and force redraw
        for (int p = 0; p < page_cnt; p++)
            hide_page(p, true);
    } else {
        // old 'active_page' is becoming inactive - hide it
        hide_page(active_page);
    }

    // switch pages
    active_page = page_num;

    // new 'active_page' is becoming active - show it
    show_page(active_page);
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
    fb.brightness(100);
    printf("framebuffer ready");
    //printf(" (spi @ %d Hz)\n", fb_spi_baud_actual);
    printf("\n");

    nav_click(0); // start out on page 0

    // initialize touchscreen
    ts_i2c_baud_actual = i2c_dev.baud();
    assert(ts.init());
    ts.set_rotation(Touchscreen::Rotation::landscape);
    printf("touchscreen ready");
    //printf(" (i2c @ %u Hz)\n", ts_i2c_baud_actual);
    printf("\n");

    sleep_ms(100);

    printf("> ");

    while (true) {

        int c = stdio_getchar_timeout_us(0);
        if (0 <= c && c <= 255 && argv.add_char(char(c))) {
            // argc/argv ready

            argv.reset();
        }

        Event event(ts.get_event());
        if (event.type == Event::Type::none)
            continue;

        // anyone have focus?
        if (GuiWidget::focus != nullptr) {
            // yes, send event there
            GuiWidget::focus->event(event);
        } else {
            // no, see if anyone wants it
            bool handled = false;
            // nav buttons?
            for (int p = 0; p < page_cnt && !handled; p++)
                handled = navs[p]->event(event);
            // anyone on current page want it?
            if (!handled)
                pages[active_page]->event(event);
        }

    } // while (true)

    sleep_ms(100);

    return 0;

} // int main()
