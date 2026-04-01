
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
#include "config.h"
#include "desktop_layout.h"
#include "locos.h"
#include "sensor.h"
#include "turnout.h"

///// Locos //////////////////////////////////////////////////////////////////

static constexpr int loco_id = 3;

static const Loco *loco = nullptr;

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

static int car_len_mm = 150; // boxcar and tanker

static void init();
static void loop(int32_t for_us = 0);

static void fetch();
static void uncouple();
static void spot();
static void home();


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
    printf("uncouple_test\n");
    printf("\n");

    init();

    turnout[0].set(true); // straight

    // let the loco charge some before trying to move
    uint32_t track_on_us = time_us_32();
    const uint32_t charge_us = 5'000'000;
    const uint32_t delay_us = charge_us - (time_us_32() - track_on_us);
    loop(delay_us);

    while (true) {
        fetch();
        uncouple();
        spot();
        home();
    }

    sleep_ms(100);

    return 0;

} // main()


static void loop(int32_t for_us)
{
    int32_t end_us = int32_t(time_us_32()) + for_us;
    while (end_us - int32_t(time_us_32()) >= 0) {
        SysLed::loop();
    }
} // loop


// On entry:
// * Loco left of uncoupler
// * Car one loco-length to the right of the uncoupler
// On return:
// * Loco+car right of uncoupler, coupled
static void fetch()
{
    printf("fetch\n");

    if (sensor_unc())
        printf("uncoupler sensor active is unexpected\n");

    // slow to uncoupler
    DccApi::loco_speed_set(loco_id, -loco->speed_dcc(slow_mms));
    while (!sensor_unc())
        loop();
    // rear of loco is at uncoupler
    DccApi::loco_speed_set(loco_id, -loco->speed_dcc(creep_mms));
    while (sensor_unc())
        loop();
    // front of loco has cleared uncoupler
    loop(mm_to_us(10, creep_mms));
    DccApi::loco_speed_set(loco_id, stop);
    loop(1'000'000);

    if (sensor_unc())
        printf("uncoupler sensor active is unexpected\n");
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
        printf("uncoupler sensor active is unexpected\n");

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
        DccApi::loco_speed_set(loco_id, -loco->speed_dcc(creep_mms));
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
            printf("uncouple failed, retrying...\n");

    } while (sensor_unc());

    if (sensor_unc())
        printf("uncoupler sensor active is unexpected\n");
}


// On entry:
// * Loco should be left of uncoupler, coupler clear of magnet
// * Car should be right of uncoupler, with its coupler over the magnet
// On return:
// * Loco & car right of uncoupler, not coupled
static void spot()
{
    printf("spot\n");

    if (sensor_unc())
        printf("uncoupler sensor active is unexpected\n");

    // creep back until rear of loco gets to sensor
    DccApi::loco_speed_set(loco_id, -loco->speed_dcc(creep_mms));
    while (!sensor_unc())
        loop();

    // continue until loco clears uncoupler
    while (sensor_unc())
        loop();

    DccApi::loco_speed_set(loco_id, stop);
    loop(1'000'000);

    if (sensor_unc())
        printf("uncoupler sensor active is unexpected\n");
}


// On entry:
// * Loco & car right of uncoupler, not coupled
// On return:
// * Loco left of uncoupler, about half a car length past it
// * Car right of uncoupler, about a loco-lenth away
static void home()
{
    printf("home\n");
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(medium_mms));
    while (!sensor_unc())
        loop();
    DccApi::loco_speed_set(loco_id, loco->speed_dcc(slow_mms));
    while (sensor_unc())
        loop();
    // go another 60 mm and stop
    int more_mm = 60 - loco->stop_mm(slow_mms);
    if (more_mm > 0)
        loop(mm_to_us(more_mm, slow_mms));
    DccApi::loco_speed_set(loco_id, stop);
    loop(1'000'000);
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
        printf("%s ... ", DccApi::status(s));
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

    uint32_t sn;
    while ((s = Loco::read_sn(loco_id, sn)) != Status::Ok) {
        printf("%s ... ", DccApi::status(s));
        loop(500'000);
    }
    printf("sn = %lu\n", sn);

    loco = Loco::find_loco(sn);
    assert(loco != nullptr);
    printf("loco: %s\n", loco->name);

    DccApi::loco_cv_val_set(loco_id, 3, 0);
    DccApi::loco_cv_val_set(loco_id, 4, 0);
    DccApi::loco_cv_val_set(loco_id, 63, loco->v_master);
    DccApi::loco_cv_bit_set(loco_id, 29, 2, 0);  // disable DC
    DccApi::loco_cv_bit_set(loco_id, 124, 2, 0); // disable startup delay

} // init
