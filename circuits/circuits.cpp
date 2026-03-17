
#include <cassert>
#include <cstdint>
#include <cstdio>
// pico
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
// misc
#include "sys_led.h"
// dcc
#include "dcc_api.h"
using Status = DccApi::Status;
// railroad
#include "afunc.h"
#include "sensor.h"
#include "turnout.h"
//
#include "locos.h"
#include "config.h"

///// Turnouts ///////////////////////////////////////////////////////////////

static constexpr int turnout_max = 2;

static Turnout turnout[turnout_max] = {
    Turnout(t0a_gpio, t0b_gpio),
    Turnout(t1a_gpio, t1b_gpio),
};

///// Sensors ////////////////////////////////////////////////////////////////

static constexpr int sensor_max = 8;

static Sensor sensor[sensor_max] = {
    Sensor(s0_gpio), Sensor(s1_gpio), Sensor(s2_gpio), Sensor(s3_gpio),
    Sensor(s4_gpio), Sensor(s5_gpio), Sensor(s6_gpio), Sensor(s7_gpio),
};

///// Async Functions ////////////////////////////////////////////////////////

static AFunc afunc;

///// Locos //////////////////////////////////////////////////////////////////

static constexpr int loco_id = 3;

//static Loco *loco = nullptr;

static const int zippy = 80;
static const int fast = 40;
static const int medium = 30;
static const int slow = 20;
static const int creep = 5;
static const int stop = 0;

// How long to go a given distance at a given speed
// For ML-8, speed_mms is about 3.5 * speed_dcc
//
//  dist_mm        s           d/s     1'000'000 us   dist_mm * 1'000'000
//  ------- x ----------- x -------- x ------------ = -------------------
//            speed_dcc d   3.5 mm/s       1 s          speed_dcc * 3.5
//
static uint32_t mm_to_us(int dist_mm, int speed_dcc)
{
    return (dist_mm * 285714) / speed_dcc;
}

// value >= 0 means set the cv to that value
static constexpr int cv_show = -1; // don't change, but read and show
static constexpr int cv_none = -2; // don't change, don't read

// set any of these to -1 to use (and show) the default
static int accel = 0; // cv_show;      // default = 20 (SP2265), 16 (Plymouth)
static int decel = 0; // default = 20 (SP2265), 12 (Plymouth)
static int master_vol = cv_none; // default = 128 (SP2265)

static void init();
static void loop(int32_t for_us = 0);

static constexpr uint32_t svc_err_delay_us = 500'000;

[[maybe_unused]] static void toots(uint32_t on1_us = 0, uint32_t off1_us = 0,
                                   uint32_t on2_us = 0, uint32_t off2_us = 0,
                                   uint32_t on3_us = 0);
[[maybe_unused]] static bool check_setup(int &spur_a, int &spur_b);
[[maybe_unused]] static void home();
[[maybe_unused]] static void spur(int sensor_num);
[[maybe_unused]] static void uncouple();
[[maybe_unused]] static void spot(int spur_num);
[[maybe_unused]] static void fetch(int spur_num);
[[maybe_unused]] static void speed_check(int speed);

[[maybe_unused]] static DccApi::Status svc_read_sn(uint32_t &sn);


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

#if 1
    int spur_a, spur_b;
    while (!check_setup(spur_a, spur_b)) {
        // flash led
        SysLed::on();
        loop(100'000);
        SysLed::off();
        loop(100'000);
    }
    int spur_e = 6 - spur_a - spur_b; // 1+2+3=6
    while (true) {
        fetch(spur_a);
        loop(2'000'000);
        uncouple();
        loop(2'000'000);
        spot(spur_e);
        loop(2'000'000);
        spur_a = spur_b;
        spur_b = spur_e;
        spur_e = 6 - spur_a - spur_b; // 1+2+3=6
        home();
        loop(3'000'000);
    }
#else
    int spur_num = 1;
    while (true) {
        uncouple();
        loop(2'000'000);
        spot(spur_num);
        loop(2'000'000);
        home();
        loop(3'000'000);
        fetch(spur_num);
        loop(2'000'000);
        if (++spur_num > 3)
            spur_num = 1;
    }
#endif

    sleep_ms(100);

    return 0;

} // main()


static void loop(int32_t for_us)
{
    int32_t end_us = int32_t(time_us_32()) + for_us;
    while (end_us - int32_t(time_us_32()) >= 0) {
        SysLed::loop();
        afunc.loop();
    }
} // loop


// sensor number 50 mm from end of spur
[[maybe_unused]]
static int sensor_50(int spur_num)
{
    assert(spur_num == 1 || spur_num == 2 || spur_num == 3);

    if (spur_num == 1)
        return 2;
    else if (spur_num == 2)
        return 4;
    else // (spur_num == 3)
        return 6;
}


// sensor number 100 mm from end of spur
[[maybe_unused]]
static int sensor_100(int spur_num)
{
    assert(spur_num == 1 || spur_num == 2 || spur_num == 3);

    if (spur_num == 1)
        return 3;
    else if (spur_num == 2)
        return 5;
    else // (spur_num == 3)
        return 7;
}


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


// Two and only two of the 100mm sensors should be active
// None of the 50mm sensors should be active
static bool check_setup(int &spur_a, int &spur_b)
{
    bool ok = true;
    spur_a = 0;
    spur_b = 0;

    for (int s = 1; s <= 3; ++s) {
        if (sensor[sensor_50(s)]) {
            printf("check_setup: ERROR: 50mm sensor on spur %d active\n", s);
            ok = false;
        }
        if (sensor[sensor_100(s)]) {
            if (spur_a == 0) {
                spur_a = s;
            } else if (spur_b == 0) {
                spur_b = s;
            } else {
                assert(s == 3);
                printf("check_setup: ERROR: 100mm sensors on all spurs\n");
                ok = false;
            }
        }
    }
    if (spur_a == 0) {
        printf("check_setup: ERROR: no 100mm sensors active\n");
        ok = false;
    } else if (spur_b == 0) {
        printf("check_setup: ERROR: only one 100mm sensor active (spur %d)\n",
               spur_a);
        ok = false;
    }

    return ok;
}


// loco is right of uncoupler
// forward to uncoupler, delay, slow down, to the house, stop
static void home()
{
    printf("home ... ");
    assert(DccApi::loco_speed_set(loco_id, zippy) == Status::Ok);
    while (!sensor[1])
        loop();
    assert(DccApi::loco_speed_set(loco_id, fast) == Status::Ok);
    loop(mm_to_us(100, fast));
    assert(DccApi::loco_speed_set(loco_id, medium) == Status::Ok);
    loop(mm_to_us(100, medium));
    assert(DccApi::loco_speed_set(loco_id, slow) == Status::Ok);
    loop(mm_to_us(100, slow));
    assert(DccApi::loco_speed_set(loco_id, creep) == Status::Ok);
    while (!sensor[0])
        loop();
    assert(DccApi::loco_speed_set(loco_id, stop) == Status::Ok);
    loop(1'000'000);
    assert(DccApi::loco_func_set(loco_id, 1, true) ==
           Status::Ok); // cabin light on
    printf("ok\n");
}


[[maybe_unused]]
static void line_turnout_0(int spur_num)
{
    assert(spur_num == 1 || spur_num == 2 || spur_num == 3);

    if (spur_num == 1)
        turnout[0].set(true);  // straight
    else                       // 2 or 3
        turnout[0].set(false); // diverge
}


[[maybe_unused]]
static void line_turnout_1(int spur_num)
{
    assert(spur_num == 1 || spur_num == 2 || spur_num == 3);

    if (spur_num == 1)
        ; // nothing to do
    else if (spur_num == 2)
        turnout[1].set(false); // diverge
    else if (spur_num == 3)
        turnout[1].set(true); // straight
}


// loco is left of uncoupler
// reverse to sensor 0, delay, slow down, to the sensor, stop
static void spur(int spur_num)
{
    int sensor_num;
    if (spur_num == 1) {
        sensor_num = 3;
    } else if (spur_num == 2) {
        sensor_num = 5;
    } else if (spur_num == 3) {
        sensor_num = 7;
    } else {
        assert(false);
    }

    printf("spur %d ... ", spur_num);

    // back fast
    assert(DccApi::loco_speed_set(loco_id, -fast) == Status::Ok);

    line_turnout_0(spur_num);

    // wait to get to uncoupler
    while (!sensor[1])
        loop();

    line_turnout_1(spur_num);

    // go most of the way
    loop(5'000'000);

    // slow down
    assert(DccApi::loco_speed_set(loco_id, -slow) == Status::Ok);
    while (!sensor[sensor_num])
        loop();

    // stop
    assert(DccApi::loco_speed_set(loco_id, stop) == Status::Ok);

    printf("ok\n");
}


// Loco+car to right of uncoupler.
// Forward slow to uncoupler, to gap, a bit more to get couplers clear of magnet.
// Creep back to gap, stop a bit, pull forward to uncouple, verifying uncoupling.
static void uncouple()
{
    printf("uncouple ... ");

    // forward until nose of loco is at uncoupler
    assert(DccApi::loco_speed_set(loco_id, slow) == Status::Ok);
    while (!sensor[1])
        loop();

    // creep forward until rear of loco is at uncoupler
    assert(DccApi::loco_speed_set(loco_id, creep) == Status::Ok);
    while (sensor[1])
        loop();

    // a bit more to get couplers clear of magnet
    loop(mm_to_us(40, creep)); // ~50mm
    assert(DccApi::loco_speed_set(loco_id, stop) == Status::Ok);

    // brief pause - couplers should be clear of magnet now
    loop(1'000'000);

    // creep back until couplers are over magnet
    assert(DccApi::loco_speed_set(loco_id, -creep) == Status::Ok);
    while (sensor[1])
        loop();

    // coupler should be over magnet now
    assert(DccApi::loco_speed_set(loco_id, stop) == Status::Ok);

    loop(500'000);

    // pull forward to uncouple (should leave car behind)
    assert(DccApi::loco_speed_set(loco_id, creep) == Status::Ok);

    loop(mm_to_us(50, creep)); // ~50mm

    assert(DccApi::loco_speed_set(loco_id, stop) == Status::Ok);

    printf("ok\n");
}


// Loco should be left of uncoupler.
// Car should have its coupler over the magnet.
static void spot(int spur_num)
{
    printf("spot %d ... ", spur_num);

    line_turnout_0(spur_num);

    // creep back until loco clears uncoupler
    assert(DccApi::loco_speed_set(loco_id, -creep) == Status::Ok);
    loop(mm_to_us(100, creep)); // ~100mm
    while (sensor[1])
        loop();

    line_turnout_1(spur_num);

    // go most of the way
    assert(DccApi::loco_speed_set(loco_id, -slow) == Status::Ok);
    loop(mm_to_us(500, slow));

    // slow down and stop at sensor 100mm from end of spur
    assert(DccApi::loco_speed_set(loco_id, -creep) == Status::Ok);
    int sensor_num = sensor_100(spur_num);
    while (!sensor[sensor_num])
        loop();
    assert(DccApi::loco_speed_set(loco_id, stop) == Status::Ok);

    printf("ok\n");
}


// Loco should be in house.
// Car should be on spur, blocking the 100mm sensor but not the 50mm one.
static void fetch(int spur_num)
{
    printf("fetch %d ... ", spur_num);

    line_turnout_0(spur_num);

    // cab light off
    assert(DccApi::loco_func_set(loco_id, 1, false) == Status::Ok);

    loop(1'000'000);

    // slow out of house
    assert(DccApi::loco_speed_set(loco_id, -slow) == Status::Ok);
    loop(mm_to_us(150, slow)); // ~150mm

    line_turnout_1(spur_num);

    // medium to uncoupler
    assert(DccApi::loco_speed_set(loco_id, -medium) == Status::Ok);
    while (!sensor[1])
        loop();

    // rear of loco has reached uncoupler now; go most of the way
    loop(mm_to_us(600, medium));

    // slow back until the 50mm sensor is hit (creep might fail to couple)
    assert(DccApi::loco_speed_set(loco_id, -slow) == Status::Ok);
    int sensor_num = sensor_50(spur_num);
    while (!sensor[sensor_num])
        loop();
    assert(DccApi::loco_speed_set(loco_id, stop) == Status::Ok);

    printf("ok\n");
}


// Loco can be anywhere on main or spur1.
// Turnout0 should be lined for spur1.
// Acceleration and deceleration should be zero.
// Back fast to sensor3, creep to sensor2, stop, pause.
// Forward at specified speed.
// Report elapsed time from clearing sensor3 to clearing sensor1.
// On the desktop layout, the distance is 174 + 227 + 246 + 123 + 123/2 = 831.5mm
static void speed_check(int speed)
{
    printf("speed_check(%d) ... ", speed);

    assert(DccApi::loco_speed_set(loco_id, -fast) == Status::Ok);
    while (!sensor[3])
        loop();
    assert(DccApi::loco_speed_set(loco_id, -creep) == Status::Ok);
    while (!sensor[2])
        loop();
    assert(DccApi::loco_speed_set(loco_id, stop) == Status::Ok);
    loop(2'000'000);

    assert(DccApi::loco_speed_set(loco_id, speed) == Status::Ok);
    while (!sensor[3]) // this might already be false
        loop();
    while (sensor[3]) // waiting for loco to pass sensor3
        loop();
    uint32_t start_us = time_us_32();

    while (!sensor[1])
        loop();
    while (sensor[1]) // waiting for loco to pass sensor3
        loop();
    uint32_t elapsed_us = time_us_32() - start_us;

    assert(DccApi::loco_speed_set(loco_id, stop) == Status::Ok);

    static constexpr uint32_t dist_um = 831'500;
    uint32_t elapsed_ms = (elapsed_us + 500) / 1000;
    uint32_t speed_mms = dist_um / elapsed_ms;

    printf("%lu ms; %lu mm/s\n", elapsed_ms, speed_mms);
}


[[maybe_unused]] static void ops_set_cv(int num, int val, const char *name);
[[maybe_unused]] static void svc_set_cv(int num, int val, const char *name);

static void init()
{
    Status s;

    Turnout::init(tp_gpio);

    DccApi::init(dcc_sig_gpio, dcc_pwr_gpio, dcc_adc_gpio, dcc_rcom_gpio,
                 dcc_rcom_uart);

    printf("reset loco...");
    while ((s = DccApi::cv_val_set(8, 8)) != Status::Ok) {
        printf("%s.", DccApi::status(s));
        loop(svc_err_delay_us);
    }
    printf("ok\n");

#if 0
    printf("read loco SN...");
    uint32_t sn;
    while ((s = svc_read_sn(sn)) != Status::Ok) {
        printf("%s.", DccApi::status(s));
        loop(svc_err_delay_us);
    }
    printf("%lu\n", sn);

    loco = get_loco(sn);
    assert(loco != nullptr);
    printf("loco: %s\n", loco->name);
#endif

    printf("create loco...");
    assert(DccApi::loco_create(loco_id) == Status::Ok);
    printf("ok\n");

#if 0 // use railcom

    printf("track on...");
    assert(DccApi::track_set(true) == Status::Ok);
    printf("ok\n");

    loop(1'000'000); // wait for loco to boot up

    // the rest is done in ops mode (assumes railcom works)

    ops_set_cv(3, accel, "acceleration");
    ops_set_cv(4, decel, "deceleration");
    ops_set_cv(63, master_vol, "master volume");

#else // don't use railcom

    svc_set_cv(3, accel, "acceleration");
    svc_set_cv(4, decel, "deceleration");
    svc_set_cv(63, master_vol, "master volume");

    loop(500'000); // wait for loco to settle?

    printf("track on...");
    assert(DccApi::track_set(true) == Status::Ok);
    printf("ok\n");

    loop(1'000'000); // wait for loco to boot up

#endif

    printf("lights on...");
    assert(DccApi::loco_func_set(loco_id, 0, true) == Status::Ok);
    printf("ok\n");

    printf("engine on...");
    assert(DccApi::loco_func_set(loco_id, 8, true) == Status::Ok);
    printf("ok\n");

} // init


static void ops_set_cv(int cv_num, int cv_val, const char *name)
{
    if (cv_val >= 0) {
        printf("set %s = %d ... ", name, cv_val);
        Status s;
        while ((s = DccApi::loco_cv_val_set(loco_id, cv_num, cv_val)) !=
               Status::Ok) {
            printf("%s.", DccApi::status(s));
            loop(1'000'000);
        }
        printf("ok\n");
    } else if (cv_val == cv_show) {
        printf("%s = ", name);
        Status s;
        while ((s = DccApi::loco_cv_val_get(loco_id, cv_num, cv_val)) !=
               Status::Ok) {
            printf("%s.", DccApi::status(s));
            loop(1'000'000);
        }
        printf("%d\n", cv_val);
    }
}


static void svc_set_cv(int cv_num, int cv_val, const char *name)
{
    if (cv_val >= 0) {
        printf("set %s = %d ... ", name, cv_val);
        Status s;
        while ((s = DccApi::cv_val_set(cv_num, cv_val)) != Status::Ok) {
            printf("%s.", DccApi::status(s));
            loop(1'000'000);
        }
        printf("ok\n");
    } else if (cv_val == cv_show) {
        printf("%s = ", name);
        Status s;
        while ((s = DccApi::cv_val_get(cv_num, cv_val)) != Status::Ok) {
            printf("%s.", DccApi::status(s));
            loop(1'000'000);
        }
        printf("%d\n", cv_val);
    }
}


static DccApi::Status svc_read_sn(uint32_t &sn)
{
    Status s;
    // railcom page
    s = DccApi::cv_val_set(31, 0);
    if (s != Status::Ok) {
        printf("svc_read_sn: %s\n", DccApi::status(s));
        return s;
    }
    s = DccApi::cv_val_set(32, 255);
    if (s != Status::Ok) {
        printf("svc_read_sn: %s\n", DccApi::status(s));
        return s;
    }
    // read SN from cv 265-268 (little-endian)
    sn = 0;
    for (int cv_num = 268; cv_num >= 265; cv_num--) {
        int cv_val;
        s = DccApi::cv_val_get(cv_num, cv_val);
        if (s != Status::Ok) {
            printf("svc_read_sn: %s\n", DccApi::status(s));
            return s;
        }
        sn = (sn << 8) | cv_val;
    }
    return Status::Ok;
}
