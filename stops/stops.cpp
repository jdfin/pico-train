
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
//
#include "config.h"
#include "locos.h"

///// Sensors ////////////////////////////////////////////////////////////////

static constexpr int sensor_max = 8;

static Sensor sensor[sensor_max] = {
    Sensor(s0_gpio), Sensor(s1_gpio), Sensor(s2_gpio), Sensor(s3_gpio),
    Sensor(s4_gpio), Sensor(s5_gpio), Sensor(s6_gpio), Sensor(s7_gpio),
};

///// Locos //////////////////////////////////////////////////////////////////

static constexpr int loco_id = 3;

static const Loco *loco = nullptr;

static void init();
static void loop(int32_t for_us = 0);
static bool check_setup();
static void stop_test(int dec, int speed_mms);
static void ops_cv_val_set(int cv_num, int cv_val);
static void ops_cv_bit_set(int cv_num, int b_num, int b_val);
static DccApi::Status ops_read_sn(uint32_t &sn);

///// Tests //////////////////////////////////////////////////////////////////

static int backup_mms = 100;

static int speed_mms[] = {100, 75, 50, 25};
static constexpr int speed_cnt = sizeof(speed_mms) / sizeof(speed_mms[0]);

static int dec[] = {0, 2, 5, 10};
static constexpr int dec_cnt = sizeof(dec) / sizeof(dec[0]);


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
    printf("stops\n");
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
        for (int si = 0; si < speed_cnt; si++) {
            for (int di = 0; di < dec_cnt; di++) {
                stop_test(dec[di], speed_mms[si]);
                int c;
                do {
                    c = stdio_getchar_timeout_us(0);
                } while (c < 0 || c > 255);
            }
        }
    }

    sleep_ms(100);

    return 0;

} // main()


static void init()
{
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
    assert(DccApi::track_set(true) == Status::Ok);
    printf("ok\n");

    loop(1'000'000); // wait for loco to boot up

    uint32_t sn;
    while ((s = ops_read_sn(sn)) != Status::Ok) {
        printf("%s.", DccApi::status(s));
        loop(500'000);
    }
    printf("sn = %lu\n", sn);

    loco = Loco::find_loco(sn);
    assert(loco != nullptr);
    printf("loco: %s\n", loco->name);

    ops_cv_val_set(3, 0);
    ops_cv_val_set(4, 0);
    ops_cv_val_set(63, loco->v_master);
    ops_cv_bit_set(29, 2, 0);  // disable DC
    ops_cv_bit_set(124, 2, 0); // disable startup delay

} // init


static void loop(int32_t for_us)
{
    int32_t end_us = int32_t(time_us_32()) + for_us;
    while (end_us - int32_t(time_us_32()) >= 0)
        SysLed::loop();
}


// Loco should be in front of the uncoupler sensor.
// Don't care about the other sensors.
static bool check_setup()
{
    if (!sensor[1]) {
        printf("ERROR: loco should be in front of the uncoupler sensor\n");
        return false;
    } else {
        return true;
    }
}


// Loco is on or left of uncoupler.
// Turnout0 should be (manually) lined for spur1.
// Acceleration should be zero.
// Deceleration will be set here.
// Back fast a little way.
// Forward at specified speed.
// When sensor1 detects loco, stop.
// Pause to measure stopping distance.
static void stop_test(int dec, int speed_mms)
{
    printf("stop_test: dec=%d speed=%d\n", dec, speed_mms);

    // back up a bit
    ops_cv_val_set(4, 0); // deceleration
    DccApi::loco_speed_set(loco_id, -loco->speed_dcc(backup_mms));
    while (!sensor[1])
        loop(); // loop here if prev test got us left of uncoupler
    while (sensor[1])
        loop(); // loop here to get right of uncoupler
    loop(1'000'000);
    DccApi::loco_speed_set(loco_id, 0);
    loop(1'000'000);

    // charge!
    ops_cv_val_set(4, dec); // deceleration we're testing
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(speed_mms));
    while (!sensor[1])
        loop();
    DccApi::loco_speed_set(loco_id, 0);

    loop(2'000'000); // pause to measure stopping distance

} // stop_test


static void ops_cv_val_set(int cv_num, int cv_val)
{
    //printf("cv%d = %d ... ", cv_num, cv_val);
    while (true) {
        Status s = DccApi::loco_cv_val_set(loco_id, cv_num, cv_val);
        if (s == Status::Ok)
            break;
        //printf("%s ... ", DccApi::status(s));
        loop(1'000'000);
    }
    //printf("ok\n");
}


static void ops_cv_bit_set(int cv_num, int b_num, int b_val)
{
    //printf("cv%d[%d] = %d ... ", cv_num, b_num, b_val);
    while (true) {
        Status s = DccApi::loco_cv_bit_set(loco_id, cv_num, b_num, b_val);
        if (s == Status::Ok)
            break;
        //printf("%s ... ", DccApi::status(s));
        loop(1'000'000);
    }
    //printf("ok\n");
}


static DccApi::Status ops_read_sn(uint32_t &sn)
{
    // set cv31/cv32 for railcom page
    ops_cv_val_set(31, 0);
    ops_cv_val_set(32, 255);

    // read sn from cv 265-268 (little-endian)
    sn = 0;
    for (int cv_num = 268; cv_num >= 265; cv_num--) {
        int cv_val;
        Status s = DccApi::loco_cv_val_get(loco_id, cv_num, cv_val);
        if (s != Status::Ok) {
            printf("ops_read_sn: %s\n", DccApi::status(s));
            return s;
        }
        printf("cv%d = %d\n", cv_num, cv_val);
        sn = (sn << 8) | cv_val;
    }

    return Status::Ok;
}
