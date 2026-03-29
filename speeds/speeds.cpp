
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
#include "sensor.h"
#include "turnout.h"
//
#include "config.h"
#include "locos.h"

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

///// Locos //////////////////////////////////////////////////////////////////

static constexpr int loco_id = 3;

static const Loco *loco = Loco::find_loco("ML560");

static void loop(int32_t for_us = 0);
static void ops_cv_set(int cv_num, int cv_val, const char *name);
static void init();
static bool check_setup();
[[maybe_unused]] static void dcc_measure(int speed_dcc);
[[maybe_unused]] static void mms_measure(int speed_mms);

static uint32_t mm_to_us(int dist_mm, int speed_dcc)
{
    return (dist_mm * 285714) / speed_dcc;
}


int main()
{
    stdio_init_all();
    SysLed::init();

    SysLed::pattern(50, 950);

#if 1
    while (!stdio_usb_connected()) {
        SysLed::loop();
        tight_loop_contents();
    }
    sleep_ms(10); // small delay needed or we lose the first prints
#endif

    SysLed::off();

    printf("\n");
    printf("speeds\n");
    printf("\n");

    init();

    while (!check_setup()) {
        // flash led waiting for correct setup
        SysLed::on();
        loop(500'000);
        SysLed::off();
        loop(500'000);
    }

    while (true) {

#if 1
        for (int dcc = 5; dcc <= 125; dcc += 5) {
            //if (dcc == 5)
                //dcc_measure(7);
            //else
                dcc_measure(dcc);
            loop(2'000'000);
        }
#else
        for (int mms = 10; mms <= 300; mms += 10) {
            mms_measure(mms);
            loop(2'000'000);
        }
#endif
    }

    sleep_ms(100);

    return 0;

} // main()


static void loop(int32_t for_us)
{
    int32_t end_us = int32_t(time_us_32()) + for_us;
    while (end_us - int32_t(time_us_32()) >= 0) {
        SysLed::loop();
        BufLog::loop();
    }
} // loop


static void ops_cv_set(int cv_num, int cv_val, const char *name)
{
    printf("set %s = %d ... ", name, cv_val);
    Status s;
    while ((s = DccApi::loco_cv_val_set(loco_id, cv_num, cv_val)) !=
           Status::Ok) {
        printf("%s.", DccApi::status(s));
        loop(1'000'000);
    }
    printf("ok\n");
}


static void init()
{
    Turnout::init(tp_gpio);

    turnout[0].set(true); // straight

    DccApi::init(dcc_sig_gpio, dcc_pwr_gpio, dcc_adc_gpio, dcc_rcom_gpio,
                 dcc_rcom_uart);

    printf("reset loco ... ");
    Status s;
    while ((s = DccApi::cv_val_set(8, 8)) != Status::Ok) {
        printf("%s.", DccApi::status(s));
        loop(500'000);
    }
    printf("ok\n");

    loop(1'000'000);

    printf("create loco ... ");
    DccApi::loco_create(loco_id);
    printf("ok\n");

    printf("track on ... ");
    DccApi::track_set(true);
    printf("ok\n");

    loop(1'000'000); // wait for loco to boot up

    ops_cv_set(3, 0, "acceleration");
    ops_cv_set(4, 0, "deceleration");

} // init


// Loco should be in front of the uncoupler sensor.
// None of the other sensors should be active.
static bool check_setup()
{
    bool ok = true;

    if (sensor[0]) {
        printf("ERROR: nothing should be in front of sensor %d\n", 0);
        ok = false;
    }

    if (!sensor[1]) {
        printf("ERROR: loco should be in front of the uncoupler sensor\n");
        ok = false;
    }

    for (int s = 2; s <= 7; s++) {
        if (sensor[s]) {
            printf("ERROR: nothing should be in front of sensor %d\n", s);
            ok = false;
        }
    }

    return ok;
}


// Loco can be anywhere on main or spur1.
// Turnout0 should be lined for spur1.
// Acceleration and deceleration should be zero.
// Back fast to sensor3, creep to sensor2, stop, pause.
// Forward at specified speed.
// Report elapsed time from clearing sensor3 to clearing sensor1.
// On the desktop layout, the distance is 174 + 227 + 246 + 123 + 123/2 = 831.5mm


// Run at various DCC speed settings, measuring actual speed in mm/s.
// Results used to fill in speeds table in locos.cpp.
static void dcc_measure(int speed_dcc)
{
    printf("dcc_measure(%d) ... ", speed_dcc);

    DccApi::loco_speed_set(loco_id, -50);
    while (!sensor[3])
        loop();
    DccApi::loco_speed_set(loco_id, -5);
    while (!sensor[2])
        loop();
    // another inch, then stop
    loop(mm_to_us(25, 5));
    DccApi::loco_speed_set(loco_id, 0);
    loop(2'000'000);

    DccApi::loco_speed_set(loco_id, speed_dcc);
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

    DccApi::loco_speed_set(loco_id, 0);

    static constexpr uint32_t dist_um = 831'500;
    uint32_t elapsed_ms = (elapsed_us + 500) / 1000;
    uint32_t speed_mms = dist_um / elapsed_ms;

    printf("%lu ms; measured %lu mm/s\n", elapsed_ms, speed_mms);
}


static void mms_measure(int speed_mms)
{
    printf("mms_measure(%d) ... ", speed_mms);

    DccApi::loco_speed_set(loco_id, -50);
    while (!sensor[3])
        loop();
    DccApi::loco_speed_set(loco_id, -5);
    while (!sensor[2])
        loop();
    // another inch, then stop
    loop(mm_to_us(25, 5));
    DccApi::loco_speed_set(loco_id, 0);
    loop(2'000'000);

    int speed_dcc = loco->speed_dcc(speed_mms);
    int actual_mms = loco->speed_mms(speed_dcc);

    DccApi::loco_speed_set(loco_id, speed_dcc);
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

    DccApi::loco_speed_set(loco_id, 0);

    static constexpr uint32_t dist_um = 831'500;
    uint32_t elapsed_ms = (elapsed_us + 500) / 1000;
    uint32_t measured_mms = dist_um / elapsed_ms;

    printf("requested %d mm/s; got %d mm/s; measured %lu mm/s\n", //
           speed_mms, actual_mms, measured_mms);
}
