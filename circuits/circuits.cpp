
#include <cassert>
#include <cstdint>
#include <cstdio>
// pico
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
// railroad
#include "config.h"
// misc
#include "buf_log.h"
#include "sys_led.h"
// ws2812
#include "ws2812.h"
// dcc
#include "dcc_api.h"
using Status = DccApi::Status;
// railroad
#include "afunc.h"
#include "desktop_layout.h"
#include "desktop_ops.h"
#include "lights.h"
#include "locos.h"
#include "sensor.h"
#include "sensor2.h"
#include "turnout.h"

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

// value >= 0 means set the cv to that value; negative values are special
static constexpr int cv_none = -1; // don't change, don't read
static constexpr int cv_show = -2; // don't change, but read and show
static constexpr int cv_bits = -3; // don't change, but read and show bits

static void ops_cv_val_set(int num, int val);
static void ops_cv_bit_set(int cv_num, int b_num, int b_val);

static void init();
static void loop(int32_t for_us = 0);
static void func_set(int f_num, bool on, bool verbose = false);

static void toots(int32_t on1_us, //
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

    const int32_t change_ms = 500;
    lights.set_change_us((change_ms * 1'000 + house_brt / 2) / house_brt);

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
    while (!check_setup(spur_a, spur_b)) {
        // flash led
        SysLed::on();
        loop(500'000);
        SysLed::off();
        loop(500'000);
    }

    // spur_e is initially empty
    int spur_e = 6 - spur_a - spur_b; // 1+2+3=6

    // Let the loco's supercap charge some before trying to move.
    // (I don't know if this really matters or helps.)
    const int32_t charge_us = 5'000'000;
    const int32_t delay_us = charge_us - (time_us_32s() - track_on_us);
    loop(delay_us);

    while (true) {

        wait_run();

        func_set(loco->f_cab_light, false);
        loop(1'000'000);

        lights.off(8'000); // turn house lights off in 8 seconds (when out of house)
        toots_backing_up();
        DesktopOps::fetch(loco_id, loco, spur_a);
        // XXX check that the fetch resulted in coupling

        do {
            loop(2'000'000);
            toots_proceeding();
            DesktopOps::uncouple(loco_id, loco);
            loop(2'000'000);
            // if spot() returns false, the car recoupled, so try again
        } while (!DesktopOps::spot(loco_id, loco, spur_e));

        toots_proceeding();

        loop(2'000'000);
        spur_a = spur_b;
        spur_b = spur_e;
        spur_e = 6 - spur_a - spur_b; // 1+2+3=6
        lights.white(5'000); // turn house lights on in 5 seconds (when approaching house)
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


static void toots(int32_t on1_us,                   //
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
static bool check_setup(int &spur_a, int &spur_b)
{
    bool ok = true;
    spur_a = 0;
    spur_b = 0;

    // If there is a car on spur1, it must be in detection range, but not too close.
    constexpr int in_range_mm = 100; // 4"
    constexpr int too_close_mm = 25; // 1"

    for (int spur = 1; spur <= 3; spur++) {
        int dist_mm = Desktop::sensor2[spur].dist_mm();
        if (dist_mm < in_range_mm) {

            printf("check_setup:");
            if (dist_mm < too_close_mm) {
                printf(" ERROR: car on spur %d is too close to end", spur);
                ok = false;
            } else {
                printf(" car detected on spur %d", spur);
            }
            printf(" at %d mm\n", dist_mm);

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
