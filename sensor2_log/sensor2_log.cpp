
#include <cassert>
#include <cmath>
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
#include "desktop_layout.h"
#include "sensor.h"
#include "sensor2.h"
#include "turnout.h"
//
#include "config.h"
#include "locos.h"

// Test for Sensor2, which measures distance from then sensor rather than just
// detect/not-detect.
//
// Loco is alone on track, either on spur1 or home track.
// Sensor2-under-test is at the end of spur1.
// Sensors #2 and #3 are at the end of spur1.
//
// The program monitors the distance measurement from the Sensor2-under-test,
// and sensors #2 and #3 for detection. The loco is backed to the right on
// spur1 until the rear of the loco is at the sensor, then forward until the
// Sensor2-under-test detects nothing.

enum class Mode {
    Poll,  // log count periodically
    Raw,   // log every count update
    Spot,  // push back and leave car
    Fetch, // back until car moves then pull out
};

// pick one
static constexpr Mode mode = Mode::Spot;

namespace ModePoll {
static void mode_poll();
}
namespace ModeRaw {
static void mode_raw();
}
namespace ModeSpot {
static void mode_spot();
}
namespace ModeFetch {
static void mode_fetch();
}

static constexpr int loco_id = 3;

static const Loco *loco = nullptr;

static constexpr int sensor2_gpio = 15; // pwm slice 7 channel B
static Sensor2 sensor2(sensor2_gpio);

static void init();
static void loop(int32_t for_us = 0);


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
    printf("sensor2_log\n");
    printf("\n");

    init(); // track, dcc, loco

    sensor2.init();

    turnout[0].set(true); // straight, mainline to spur1

    // let the loco charge some before trying to move
    uint32_t track_on_us = time_us_32();
    const uint32_t charge_us = 5'000'000;
    const uint32_t delay_us = charge_us - (time_us_32() - track_on_us);
    loop(delay_us);

    if (mode == Mode::Poll)
        ModePoll::mode_poll();
    else if (mode == Mode::Raw)
        ModeRaw::mode_raw();
    else if (mode == Mode::Spot)
        ModeSpot::mode_spot();
    else if (mode == Mode::Fetch)
        ModeFetch::mode_fetch();

    sleep_ms(100);

    return 0;

} // main()


static void loop(int32_t for_us)
{
    int32_t end_us = int32_t(time_us_32()) + for_us;
    while (end_us - int32_t(time_us_32()) >= 0)
        SysLed::loop();
}


static void init()
{
    Status s;

    Turnout::init(tp_gpio);

    DccApi::init(dcc_sig_gpio, dcc_pwr_gpio, dcc_adc_gpio, dcc_rcom_gpio,
                 dcc_rcom_uart);

    printf("Reset loco ... ");
    while ((s = DccApi::cv_val_set(8, 8)) != Status::Ok) {
        printf("%s ... ", DccApi::status(s));
        loop(500'000);
    }
    printf("ok\n");

    loop(1'000'000);

    printf("Create loco ... ");
    assert(DccApi::loco_create(loco_id) == Status::Ok);
    printf("ok\n");

    printf("Track on ... ");
    assert(DccApi::track_set(true) == Status::Ok);
    printf("ok\n");

    loop(1'000'000); // wait for loco to boot up

    uint32_t sn;
    while ((s = Loco::read_sn(loco_id, sn)) != Status::Ok) {
        printf("%s ... ", DccApi::status(s));
        loop(500'000);
    }
    printf("SN = %lu\n", sn);

    loco = Loco::find_loco(sn);
    assert(loco != nullptr);
    printf("Loco: %s\n", loco->name);

    DccApi::loco_cv_val_set(loco_id, 3, 0);
    DccApi::loco_cv_val_set(loco_id, 4, 0);
    DccApi::loco_cv_bit_set(loco_id, 29, 2, 0); // disable DC

} // init


namespace ModePoll {


// log count periodically
static void mode_poll()
{
    const uint32_t poll_interval_us = 100'000; // how often to check sensors

    uint32_t poll_us = time_us_32() + poll_interval_us;

    int speed_mms = 50;

    printf("T S3 S2 MM\n");

    while (true) {
        DccApi::loco_speed_set(loco_id, loco->speed_dcc(speed_mms));
        uint32_t now_us;
        do {
            now_us = time_us_32();
        } while (int32_t(now_us - poll_us) < 0);
        bool s1 = sensor[1];
        bool s2 = sensor[2];
        bool s3 = sensor[3];
        int d_mm = sensor2.dist_mm();
        if (s2 || s3 || d_mm >= 0) {
            static uint32_t zero_us = now_us;
            printf("%0.1f %d %d %d\n", (now_us - zero_us) / 1'000'000.0, s3, s2,
                   d_mm);
        }
        if (s2)                          // end of spur1
            speed_mms = +abs(speed_mms); // go forward
        if (s1)                          // uncoupler
            speed_mms = -abs(speed_mms); // go backward
        poll_us += poll_interval_us;
    }
}


} // namespace ModePoll


namespace ModeRaw {


// start with the first time something is detected
static uint32_t zero_us = 0;

static constexpr int log_len = 1000; // ~10 sec @ ~100 Hz max
static struct {
    uint32_t time_us;
    uint16_t count;
} raw_log[log_len];

static int log_idx = 0;

static int speed_mms = 25;


static void callback(uint16_t count, intptr_t)
{
    uint32_t now_us = time_us_32();

    if (zero_us == 0 && count >= 1990)
        return; // not started, and nothing detected yet

    if (zero_us == 0)
        zero_us = now_us;

    if (log_idx < log_len) {
        raw_log[log_idx].time_us = now_us - zero_us;
        raw_log[log_idx].count = count;
        log_idx++;
    }
}


static void mode_raw()
{
    // log every count update
    sensor2.set_callback(callback, 0);

    // drive to the end of spur1
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(-speed_mms));
    while (!sensor[2])
        loop();

    // drive about fwd_mm forward
    constexpr int fwd_mm = 500;
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(speed_mms));
    loop(fwd_mm * 1'000'000 / speed_mms);
    DccApi::loco_speed_set(loco_id, 0);
    loop(1'000'000);

    sensor2.set_callback(nullptr, 0);

    // print log
    printf("T Count Dist_mm\n");
    for (int i = 0; i < log_idx; i++) {
        float time_s = raw_log[i].time_us / 1'000'000.0;
        float dist_mm = (raw_log[i].count <= 1990)
                            ? ((raw_log[i].count - 1000) * 3 + 2) / 4.0
                            : 0.0;
        printf("%0.3f %d %0.2f\n", time_s, raw_log[i].count, dist_mm);
    }
}

} // namespace ModeRaw


namespace ModeSpot {

// Tank car ends up about 40 mm from the sensor
// Boxcar ends up about 45 mm from the sensor

static constexpr int fast_mms = 75;
static constexpr int slow_mm = 200; // slow down here (likely "first detect")
static constexpr int slow_mms = 20;
static constexpr int stop_mm = 50; // stop when sensor2 detects <= this distance


static void mode_spot()
{
    // push back and leave car

    DccApi::loco_speed_set(loco_id, loco->speed_dcc(-fast_mms));
    while (sensor2.dist_mm() > slow_mm)
        loop();

    DccApi::loco_speed_set(loco_id, loco->speed_dcc(-slow_mms));
    while (sensor2.dist_mm() > stop_mm)
        loop();

    DccApi::loco_speed_set(loco_id, 0);
    loop(2'000'000);

    // drive about fwd_mm forward
    constexpr int fwd_mm = 500;
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(slow_mms));
    loop(slow_mm * 1'000'000 / slow_mms);
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(fast_mms));
    loop((fwd_mm - slow_mm) * 1'000'000 / fast_mms);
    DccApi::loco_speed_set(loco_id, 0);
    loop(1'000'000);
}


} // namespace ModeSpot


namespace ModeFetch {


// back slowly until car moves, then pull out

static constexpr int slow_mms = 25;
static constexpr int move_mm = 15;
static constexpr int fast_mms = 50;


static void mode_fetch()
{
    // get initial reading on the car
    int dist_mm = sensor2.dist_mm();

    DccApi::loco_speed_set(loco_id, loco->speed_dcc(-slow_mms));
    while (sensor2.dist_mm() > (dist_mm - move_mm))
        loop();

    DccApi::loco_speed_set(loco_id, 0);
    loop(2'000'000);

    // drive about fwd_mm forward
    constexpr int slow_mm = 100;
    constexpr int fast_mm = 400;
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(slow_mms));
    loop(slow_mm * 1'000'000 / slow_mms);
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(fast_mms));
    loop(fast_mm * 1'000'000 / fast_mms);
    DccApi::loco_speed_set(loco_id, 0);
    loop(1'000'000);
}


} // namespace ModeFetch
