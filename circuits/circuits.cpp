
#include <cassert>
#include <cstdint>
#include <cstdio>
// pico
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
// railroad
#include "railroad/config.h"
// misc
#include "misc/buf_log.h"
#include "misc/sys_led.h"
// ws2812
#include "ws2812/ws2812.h"
// dcc
#include "dcc/dcc_api.h"
using Status = DccApi::Status;
// railroad
#include "railroad/afunc.h"
#include "railroad/desktop_layout.h"
#include "railroad/desktop_ops.h"
#include "railroad/display.h"
#include "railroad/lights.h"
#include "railroad/locos.h"
#include "railroad/sensor.h"
#include "railroad/sensor2.h"
#include "railroad/turnout.h"

static constexpr bool snd_any = true;
static constexpr bool snd_engine = snd_any && true;
static constexpr bool snd_horn = snd_any && true;
static constexpr bool snd_bell = snd_any && true;

static AFunc afunc;

static constexpr int loco_id = 3;

static const Loco *loco = nullptr;

// lights in house is a strip of ws2812; use four of them
static Ws2812 ws2812(ws2812_gpio, 4);
static constexpr int house_brt = 255;
static Lights lights(ws2812, house_brt);

// OLED display in window
static Display display;

// value >= 0 means set the cv to that value; negative values are special
static constexpr int cv_none = -1; // don't change, don't read
static constexpr int cv_show = -2; // don't change, but read and show
static constexpr int cv_bits = -3; // don't change, but read and show bits

static void ops_cv_val_set(int num, int val);
static void ops_cv_bit_set(int cv_num, int b_num, int b_val);

static void init();
static void loop(int32_t for_us = 0);
static void func_set(int f_num, bool on, bool verbose = false);

static void toots(int32_t on1_us,                          //
                  int32_t off1_us = 0, int32_t on2_us = 0, //
                  int32_t off2_us = 0, int32_t on3_us = 0);

inline int32_t time_us_32s()
{
    return int32_t(time_us_32());
}

static void toots_backing_up()
{
    if (snd_horn && loco->f_horn >= 0) {
        toots(250'000, 500'000, 250'000, 500'000, 250'000);
        loop(250'000 + 500'000 + 250'000 + 500'000 + 250'000);
        loop(1'000'000);
    }
}

static void toots_proceeding()
{
    if (snd_horn && loco->f_horn >= 0) {
        toots(750'000, 750'000, 750'000);
        loop(750'000 + 750'000 + 750'000);
        loop(1'000'000);
    }
}

static void uncoupler_sensor(bool active, intptr_t)
{
    static int32_t last_us = INT32_MAX; // last time this callback was called
    int32_t now_us = time_us_32s();
    printf("uncoupler sensor %s", active ? "active" : "inactive");
    if (last_us != INT32_MAX)
        printf(" (+%lu ms)", (now_us - last_us + 500) / 1000);
    printf("\n");
    last_us = now_us;
}

static bool sound_sw()
{
    return Desktop::sw[1];
}

static void wait_run(bool set_sound = true)
{
    bool sound = sound_sw();

    if (set_sound)
        ops_cv_val_set(63, sound ? loco->v_master : 0);

    if (Desktop::sw[0])
        return;

    // waiting for switch; house lights red, pico led blinking

    lights.red();
    SysLed::pattern(50, 950);
    display.show("Paused");

    while (!Desktop::sw[0]) {
        loop();
        if (sound != sound_sw()) {
            sound = sound_sw();
            if (set_sound)
                ops_cv_val_set(63, sound ? loco->v_master : 0);
        }
    }

    SysLed::off();
    lights.white();
}

static bool check_setup(int &spur_a, int &spur_b);


int main()
{
    stdio_init_all();
    SysLed::init();
    ws2812.init();
    lights.off();
    display.init();

    const int32_t change_ms = 500;
    lights.set_change_us((change_ms * 1'000 + house_brt / 2) / house_brt);

    display.clear();
    display.wait();

    wait_run(false);

    printf("\n");
    printf("circuits\n");
    printf("\n");

    init();

    Desktop::sensor_unc().set_callback(uncoupler_sensor, 0);

    DesktopOps::set_loop(loop);

    lights.white();

    int32_t track_on_us = time_us_32s();

    // spurs a and b initially have the cars on them
    int spur_a, spur_b;
    SysLed::pattern(500, 500);
    while (!check_setup(spur_a, spur_b))
        loop(1'000'000);
    SysLed::off();

    // spur_empty is initially empty
    int spur_empty = 6 - spur_a - spur_b; // 1+2+3=6

    // Let the loco's supercap charge some before trying to move.
    // (I don't know if this really matters or helps.)
    const int32_t charge_us = 5'000'000;
    const int32_t delay_us = charge_us - (time_us_32s() - track_on_us);
    loop(delay_us);

    char msg[20];

    while (true) {

        wait_run();

        sprintf(msg, "Fetch %d", spur_a);
        display.show(msg);

        func_set(loco->f_cab_light, false);
        loop(1'000'000);

        // turn house lights off in 8 seconds (when out of house)
        lights.off(8'000);

        toots_backing_up();
        DesktopOps::fetch(loco_id, loco, spur_a);
        // XXX check that the fetch resulted in coupling

        do {
            display.show("Uncouple");
            loop(2'000'000);
            toots_proceeding();
            DesktopOps::uncouple(loco_id, loco);
            loop(2'000'000);
            sprintf(msg, "Spot %d", spur_empty);
            display.show(msg);
            // if spot() returns false, the car recoupled, so try again
        } while (!DesktopOps::spot(loco_id, loco, spur_empty));

        toots_proceeding();
        display.show("Home");
        loop(2'000'000);
        spur_a = spur_b;
        spur_b = spur_empty;
        spur_empty = 6 - spur_a - spur_b; // 1+2+3=6
        // turn house lights on in 5 seconds (when approaching house)
        lights.white(5'000);
        DesktopOps::home(loco_id, loco);
        loop(1'000'000);
        func_set(loco->f_cab_light, true);
        loop(3'000'000);
    }

    sleep_ms(100);

    return 0;

} // main()


static void loop(int32_t for_us)
{
    // works if for_us < 0
    int32_t end_us = time_us_32s() + for_us;
    while (end_us - time_us_32s() >= 0) {
        SysLed::loop();
        lights.loop();
        afunc.loop();
        BufLog::loop();
        tight_loop_contents();
    }
} // loop


static void func_set(int f_num, bool on, bool verbose)
{
    if (f_num < 0)
        return;

    if (verbose)
        printf("f%d %s ... ", f_num, on ? "on" : "off");

    DccApi::loco_func_set(loco_id, f_num, on);

    if (verbose)
        printf("ok\n");

} // func_set


static void toots(int32_t on1_us,                  //
                  int32_t off1_us, int32_t on2_us, //
                  int32_t off2_us, int32_t on3_us)
{
    int32_t now_us = time_us_32s();
    if (on1_us > 0) {
        afunc.put(now_us, loco_id, 2, true);
        now_us += on1_us;
        afunc.put(now_us, loco_id, 2, false);
        if (on2_us > 0) {
            now_us += off1_us;
            afunc.put(now_us, loco_id, 2, true);
            now_us += on2_us;
            afunc.put(now_us, loco_id, 2, false);
            if (on3_us > 0) {
                now_us += off2_us;
                afunc.put(now_us, loco_id, 2, true);
                now_us += on3_us;
                afunc.put(now_us, loco_id, 2, false);
            }
        }
    }
} // toots


// There should be cars on two and only two spurs
//
// Display:
// +----------------------+
// |  Spur 1:   2.5"   v  |
// |  Spur 2:             |
// |  Spur 3:   0.9"      |
// |          OK          |
// +----------------------+
static bool check_setup(int &spur_a, int &spur_b)
{
    bool ok = true;
    spur_a = 0;
    spur_b = 0;

    const char check = 0x12; // checkmark in chicago-12
    const char nope = 0x15;  // X-mark in customized chicago-12

    // If there is a car on spur1, it must be in detection range, but not too close.
    constexpr int in_range_mm = 100; // 4"
    constexpr int too_close_mm = 25; // 1"

    int dist_mm[3];

    display.clear();
    constexpr int line_cnt = 4;
    constexpr int line_hgt = display.height() / line_cnt;
    int line_ctr[line_cnt] = {
        1 * line_hgt / 2,
        3 * line_hgt / 2,
        5 * line_hgt / 2,
        7 * line_hgt / 2,
    };

    for (int i = 0; i < 3; i++) {
        int spur = i + 1;
        dist_mm[i] = Desktop::sensor2[spur].dist_mm();

        display.clear(0, i * display.height() / line_cnt, display.width(),
                      line_hgt);
        char str_work[16];
        sprintf(str_work, "Spur %d", spur);
        constexpr int spur_name_col = 10; // left aligned
        constexpr int spur_dist_col =
            (display.width() * 5) / 8;                    // center aligned
        constexpr int spur_ok_col = display.width() - 10; // right aligned
        display.set_align(Display::HAlign::Left, Display::VAlign::Center);
        display.puts(spur_name_col, line_ctr[i], str_work);
        // testing against 250 instead of infinity makes sure inches fits in 3.1f
        if (dist_mm[i] < 250) {
            sprintf(str_work, "%3.1f\"", dist_mm[i] / 25.4); // sprintf rounds
            display.set_align(Display::HAlign::Center, Display::VAlign::Center);
            display.puts(spur_dist_col, line_ctr[i], str_work);
        }

        if (dist_mm[i] < in_range_mm) {

            printf("check_setup:");
            display.set_align(Display::HAlign::Right, Display::VAlign::Center);
            if (dist_mm[i] < too_close_mm) {
                printf(" ERROR: car on spur %d is too close to end", spur);
                display.putc(spur_ok_col, line_ctr[i], nope);
                ok = false;
            } else {
                printf(" car detected on spur %d", spur);
                display.putc(spur_ok_col, line_ctr[i], check);
            }
            printf(" at %d mm\n", dist_mm[i]);

            if (spur_a == 0) {
                spur_a = spur;
            } else if (spur_b == 0) {
                spur_b = spur;
            } else {
                assert(spur == 3);
                printf("check_setup: ERROR: cars detected on all spurs\n");
                ok = false;
            }
        }
    }

    if (spur_a == 0) {
        printf("check_setup: ERROR: no cars on any spurs\n");
        ok = false;
    } else if (spur_b == 0) {
        printf("check_setup: ERROR: only one car detected (spur %d)\n", spur_a);
        ok = false;
    }

    if (ok) {
        display.set_align(Display::HAlign::Center, Display::VAlign::Center);
        display.puts(display.width() / 2, line_ctr[3], "OK");
    }

    display.flush();

    return ok;
}


static void init()
{
    Desktop::init();

    DccApi::init(dcc_bit_gpio, dcc_pwr_gpio, dcc_adc_gpio, dcc_rcom_gpio,
                 dcc_rcom_uart);

    Status s;

    printf("reset loco ... ");
    while ((s = DccApi::cv_val_set(8, 8)) != Status::Ok) {
        printf("%s ... ", DccApi::status(s));
        loop(1'000'000);
    }
    printf("ok\n");

    loop(1'000'000);

    printf("create loco ... ");
    assert(DccApi::loco_create(loco_id) == Status::Ok);
    printf("ok\n");

    printf("track on ... ");
    assert(DccApi::track_set(true) == Status::Ok);
    printf("ok\n");

    loop(1'000'000); // wait for loco to boot up

    printf("read sn ... ");
    uint32_t sn;
    while ((s = Loco::read_sn(loco_id, sn)) != Status::Ok) {
        printf("%s ... ", DccApi::status(s));
        loop(1'000'000);
    }
    printf("0x%08lx\n", sn);

    loco = Loco::find_loco(sn);
    assert(loco != nullptr);
    printf("loco: %s\n", loco->name);

    ops_cv_val_set(3, 10);
    ops_cv_val_set(4, 0);
    ops_cv_val_set(63, sound_sw() ? loco->v_master : 0);
    ops_cv_val_set(29, cv_bits);
    ops_cv_bit_set(29, 2, 0); // disable DC
    ops_cv_val_set(29, cv_bits);
    ops_cv_val_set(124, cv_bits);
    ops_cv_bit_set(124, 2, 0); // disable startup delay
    ops_cv_val_set(124, cv_bits);

    if (snd_engine && loco->v_engine >= 0) {
        ops_cv_val_set(31, 16);
        ops_cv_val_set(32, 1);
        ops_cv_val_set(259, cv_show);
        ops_cv_val_set(259, loco->v_engine);
    }

    if (loco->setup != nullptr)
        loco->setup(loco_id);

    func_set(loco->f_headlight, true);
    func_set(loco->f_engine, snd_engine);

} // init


static void ops_cv_val_set(int cv_num, int cv_val)
{
    if (cv_val >= 0) {
        printf("cv%d = %d ... ", cv_num, cv_val);
        while (true) {
            Status s = DccApi::loco_cv_val_set(loco_id, cv_num, cv_val);
            if (s == Status::Ok)
                break;
            printf("%s ... ", DccApi::status(s));
            loop(1'000'000);
        }
        printf("ok\n");
    } else if (cv_val == cv_show || cv_val == cv_bits) {
        printf("cv%d = ", cv_num);
        int val;
        while (true) {
            Status s = DccApi::loco_cv_val_get(loco_id, cv_num, val);
            if (s == Status::Ok)
                break;
            printf("%s ... ", DccApi::status(s));
            loop(1'000'000);
        }
        if (cv_val == cv_show) {
            printf("%d\n", val);
        } else { // cv_val == cv_bits
            for (int b = 7; b >= 0; b--)
                printf("%d", (val >> b) & 1);
            printf("\n");
        }
    }
} // ops_cv_val_set


static void ops_cv_bit_set(int cv_num, int b_num, int b_val)
{
    if (b_num >= 0) {
        printf("cv%d[%d] = %d ... ", cv_num, b_num, b_val);
        while (true) {
            Status s = DccApi::loco_cv_bit_set(loco_id, cv_num, b_num, b_val);
            if (s == Status::Ok)
                break;
            printf("%s ... ", DccApi::status(s));
            loop(1'000'000);
        }
        printf("ok\n");
    } else if (b_num == cv_show) {
        printf("cv%d[%d] = ", cv_num, b_num);
        while (true) {
            int cv_val;
            Status s = DccApi::loco_cv_val_get(loco_id, cv_num, cv_val);
            if (s == Status::Ok) {
                b_val = (cv_val >> b_num) & 1;
                break;
            }
            printf("%s ... ", DccApi::status(s));
            loop(1'000'000);
        }
        printf("%d\n", b_val);
    }
} // ops_cv_bit_set
