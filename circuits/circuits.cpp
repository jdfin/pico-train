
#include <cassert>
#include <cstdint>
#include <cstdio>
// pico
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
// misc
#include "buf_log.h"
#include "sys_led.h"
// dcc
#include "dcc_api.h"
using Status = DccApi::Status;
// railroad
#include "afunc.h"
#include "config.h"
#include "desktop_layout.h"
#include "locos.h"
#include "sensor.h"
#include "sensor2.h"
#include "turnout.h"

static constexpr bool snd_engine = true;
static constexpr bool snd_horn = true;
static constexpr bool snd_bell = true;

static AFunc afunc;

static constexpr int loco_id = 3;

static const Loco *loco = nullptr;

static const int zippy_mms = 300;
static const int fast_mms = 150;
static const int medium_mms = 100;
static const int slow_mms = 75;
static const int creep_mms = 25;
static const int stop = 0;

// How many microseconds to go dist_mm at speed_mms
static uint32_t mm_to_us(int dist_mm, int speed_mms)
{
    const uint32_t t_us = (dist_mm * 1'000'000 + speed_mms / 2) / speed_mms;
    return t_us;
}

// value >= 0 means set the cv to that value; negative values are special
static constexpr int cv_none = -1; // don't change, don't read
static constexpr int cv_show = -2; // don't change, but read and show
static constexpr int cv_bits = -3; // don't change, but read and show bits

static int car_len_mm = 150; // boxcar and tanker

static void ops_cv_val_set(int num, int val);
static void ops_cv_bit_set(int cv_num, int b_num, int b_val);

static void init();
static void loop(int32_t for_us = 0);
static void func_set(int f_num, bool on, bool verbose = false);

static void toots(uint32_t on1_us, uint32_t off1_us = 0, uint32_t on2_us = 0,
                  uint32_t off2_us = 0, uint32_t on3_us = 0);

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

static bool check_setup(int &spur_a, int &spur_b);
static void fetch(int spur_num);
static void uncouple();
static bool spot(int spur_num);
static void home();


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
    printf("circuits\n");
    printf("\n");

    init();

    uint32_t track_on_us = time_us_32();

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

    // let the supercap charge some before trying to move
    const uint32_t charge_us = 5'000'000;
    const uint32_t delay_us = charge_us - (time_us_32() - track_on_us);
    loop(delay_us);

    while (true) {

        fetch(spur_a);

        do {
            loop(2'000'000);
            uncouple();
            loop(2'000'000);
            // if spot() returns false, the car recoupled, so try again
        } while (!spot(spur_e));

        loop(2'000'000);
        spur_a = spur_b;
        spur_b = spur_e;
        spur_e = 6 - spur_a - spur_b; // 1+2+3=6
        home();
        loop(3'000'000);
    }

    sleep_ms(100);

    return 0;

} // main()


static void loop(int32_t for_us)
{
    int32_t end_us = int32_t(time_us_32()) + for_us;
    while (end_us - int32_t(time_us_32()) >= 0) {
        SysLed::loop();
        afunc.loop();
        BufLog::loop();
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


static void toots(uint32_t on1_us,                   //
                  uint32_t off1_us, uint32_t on2_us, //
                  uint32_t off2_us, uint32_t on3_us)
{
    uint32_t now_us = time_us_32();
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
        int dist_mm = sensor2[spur].dist_mm();
        if (dist_mm < in_range_mm) {

            printf("check_setup:");
            if (dist_mm < too_close_mm) {
                printf(" ERROR: car on spur %d is too close to end", spur);
                ok = false;
            } else {
                printf("car detected on spur %d", spur);
            }
            printf(" (%d mm)\n", dist_mm);

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


// Loco should be in house.
// Car should be on spur, in view of the sensor but not too close to it.
static void fetch(int spur_num)
{
    printf("fetch %d\n", spur_num);

    line_turnout_0(spur_num);

    func_set(loco->f_cab_light, false);

    loop(1'000'000);

    toots_backing_up();

    // slow out of house
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(-slow_mms));
    loop(mm_to_us(150, slow_mms));

    line_turnout_1(spur_num);

    // medium to uncoupler
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(-medium_mms));
    while (!sensor_unc())
        loop();

    // Rear of loco has reached uncoupler now; go most of the way.
    // If the car is 100 mm from the end, and is car_len_mm long, this should
    // get us to 100 mm from the car.
    int most_mm = unc_to_spur_mm(spur_num) - 100 - car_len_mm - 100;
    loop(mm_to_us(most_mm, medium_mms));

    // creep back until we get the car (until the car moves)
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(-creep_mms));
    const int dist_mm = sensor_spur(spur_num).dist_mm();
    printf("fetch 1: start at %d mm\n", dist_mm);
    constexpr int move_mm = 15;
    while (sensor_spur(spur_num).dist_mm() > (dist_mm - move_mm))
        loop();

    DccApi::loco_speed_set(loco_id, stop);
    loop(1'000'000);

    printf("fetch 1: moved to %d mm\n", sensor_spur(spur_num).dist_mm());

    if (loco->f_clank >= 0) {
        func_set(loco->f_clank, true);
        loop(1'000'000);
        func_set(loco->f_clank, false);
    }
    loop(1'000'000);
}


// On entry:
// * Loco+car right of uncoupler, coupled
// On return:
// * Loco left of uncoupler, coupler clear of magnet
// * Car just right of uncoupler with coupler over magnet
static void uncouple()
{
    printf("uncouple\n");

    if (sensor_unc())
        printf("unexpected: uncoupler sensor is active\n");

    toots_proceeding();

    // forward until nose of loco is at uncoupler (might already be there)
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(slow_mms));
    while (!sensor_unc())
        loop();

    // creep forward until rear of loco (gap) is at uncoupler
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(creep_mms));
    while (sensor_unc())
        loop();

    // a bit more to get couplers clear of magnet (~50 mm)
    int more_mm = 50 - loco->stop_mm(creep_mms);
    if (more_mm > 0)
        loop(mm_to_us(more_mm, creep_mms));
    DccApi::loco_speed_set(loco_id, stop);
    loop(1'000'000);

    // couplers should be clear of magnet now

    do {

        // creep back until couplers are over magnet
        DccApi::loco_speed_set(loco_id, loco->speed_dcc(-creep_mms));
        while (sensor_unc())
            loop();
        DccApi::loco_speed_set(loco_id, stop);
        loop(500'000);

        // couplers should be over magnet now

        // pull forward to uncouple (should leave car behind)
        DccApi::loco_speed_set(loco_id, loco->speed_dcc(creep_mms));
        loop(mm_to_us(car_len_mm / 2, creep_mms));
        DccApi::loco_speed_set(loco_id, stop);
        loop(500'000);

        // retry if necessary
        if (sensor_unc())
            printf("uncouple failed! retrying...\n");

    } while (sensor_unc());

    if (sensor_unc())
        printf("unexpected: uncoupler sensor is active\n");

    printf("uncouple done\n");
}


// On entry:
// * Loco should be left of uncoupler, coupler clear of magnet
// * Car should be right of uncoupler, with its coupler over the magnet
// On return:
// * Loco & car on spur, not coupled
// * Loco creeping forward
// Errors:
// * Sometimes the cars recouple on the way back (esp. the tank car to
//   spur 2). If that happens, the 110 mm sensor goes inactive)
// Return:
// *  true if the car was left behind
// *  false if the car is still coupled
static bool spot(int spur_num)
{
    printf("spot %d\n", spur_num);

    line_turnout_0(spur_num);

    if (snd_bell)
        func_set(loco->f_bell, true);

    loop(1'000'000);

    // creep back until loco clears uncoupler
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(-creep_mms));
    loop(mm_to_us(100, creep_mms));
    while (sensor_unc())
        loop();

    line_turnout_1(spur_num);

    // Spur 2 has an s-turn that can cause a recoupling or even derail, both
    // observed with UP852 and the tank car, but never (yet) with any other
    // loco or the boxcar.
    if (spur_num != 2) {
        // spur 1 or 3, a bit faster most of the way
        // subtract loco and car len, plan to leave it 100 mm from the end,
        // and we'll start creeping 100 mm before that
        int slow_mm =
            unc_to_spur_mm(spur_num) - loco->len_mm - car_len_mm - 100 - 100;
        DccApi::loco_speed_set(loco_id, loco->speed_dcc(-slow_mms));
        loop(mm_to_us(slow_mm, slow_mms));
    }

    DccApi::loco_speed_set(loco_id, loco->speed_dcc(-creep_mms));

    // stop when close enough to the end
    constexpr int stop_mm = 75; // stop this far from the sensor
    int detected_mm = 0;        // where sensor first detected car
    int last_mm = 0;            // last reading before stopping
    do {
        last_mm = sensor_spur(spur_num).dist_mm();
        if (detected_mm == 0 && last_mm <= 500)
            detected_mm = last_mm;
        loop();
    } while (last_mm > stop_mm);

    DccApi::loco_speed_set(loco_id, stop);
    loop(1'000'000);

    if (snd_bell)
        func_set(loco->f_bell, false);

    printf("spot %d: detected at %d mm, stopped at %d mm, left at %d mm\n",
           spur_num, detected_mm, last_mm, sensor_spur(spur_num).dist_mm());

    if (loco->f_clank >= 0) {
        func_set(loco->f_clank, true);
        loop(1'000'000);
        func_set(loco->f_clank, false);
    }
    loop(1'000'000);

    toots_proceeding();

    // Make sure the car is left behind; creep ahead a bit and make sure
    // the car does not move.
    int dist_mm = sensor_spur(spur_num).dist_mm();
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(creep_mms));
    loop(mm_to_us(30, creep_mms));
    // If the car did not move too much, it is not coupled; return true.
    return (sensor_spur(spur_num).dist_mm() - dist_mm) < 15;
}


// loco is right of uncoupler
// Forward to uncoupler, delay, slow down, to the house, stop.
// The house sensor usually sees the far end of the house at ~200 mm, so
// ignore readings until we get close.
static void home()
{
    printf("home\n");
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(zippy_mms));
    while (!sensor_unc())
        loop();

    DccApi::loco_speed_set(loco_id, loco->speed_dcc(fast_mms));
    loop(mm_to_us(75, fast_mms));

    DccApi::loco_speed_set(loco_id, loco->speed_dcc(medium_mms));
    loop(mm_to_us(100, medium_mms));

    DccApi::loco_speed_set(loco_id, loco->speed_dcc(slow_mms));
    constexpr int creep_at_mm = 150;
    while (sensor_home().dist_mm() > creep_at_mm)
        loop();

    DccApi::loco_speed_set(loco_id, loco->speed_dcc(creep_mms));
    constexpr int stop_at_mm = 35;
    while (sensor_home().dist_mm() > stop_at_mm)
        loop();

    DccApi::loco_speed_set(loco_id, stop);
    loop(1'000'000);

    func_set(loco->f_cab_light, true);
}


static void init()
{
    Status s;

    for (int i = 0; i < sensor_max; i++)
        sensor[i].init();

    for (int i = 0; i < sensor2_max; i++)
        sensor2[i].init();

    Turnout::init(tp_gpio);

    DccApi::init(dcc_sig_gpio, dcc_pwr_gpio, dcc_adc_gpio, dcc_rcom_gpio,
                 dcc_rcom_uart);

    printf("reset loco ... ");
    while ((s = DccApi::cv_val_set(8, 8)) != Status::Ok) {
        printf("%s.", DccApi::status(s));
        loop(500'000);
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
    printf("%lu\n", sn);

    loco = Loco::find_loco(sn);
    assert(loco != nullptr);
    printf("loco: %s\n", loco->name);

    ops_cv_val_set(3, 10);
    ops_cv_val_set(4, 0);
    ops_cv_val_set(63, loco->v_master);
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
