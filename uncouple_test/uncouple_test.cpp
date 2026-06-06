
#include <cassert>
#include <cstdint>
#include <cstdio>
// pico
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
// misc
#include "misc/sys_led.h"
// dcc
#include "dcc/dcc_api.h"
using Status = DccApi::Status;
// railroad
#include "railroad/config.h"
#include "railroad/desktop_layout.h"
#include "railroad/desktop_ops.h"
#include "railroad/locos.h"
#include "railroad/sensor.h"
#include "railroad/turnout.h"

static constexpr int loco_id = 3;

static const Loco *loco = nullptr;

// How many microseconds to go dist_mm at speed_mms
static uint32_t mm_to_us(int dist_mm, int speed_mms)
{
    const uint32_t t_us = (dist_mm * 1'000'000 + speed_mms / 2) / speed_mms;
    return t_us;
}

static void uncoupler_sensor(bool active, intptr_t)
{
    printf("uncoupler sensor %s\n", active ? "active" : "inactive");
}

static void init();
static void loop(int32_t for_us = 0);
static void fetch();
static void spot();
static void home();


int main()
{
    stdio_init_all();
    SysLed::init();

    SysLed::pattern(50, 950);

    while (!stdio_usb_connected()) {
        SysLed::loop();
        tight_loop_contents();
    }

    sleep_ms(10);

    SysLed::off();

    printf("\n");
    printf("uncouple_test\n");
    printf("\n");

    init();

    DesktopOps::set_loop(loop);

    Desktop::sensor_unc().set_callback(uncoupler_sensor, 0);

    Desktop::line_turnout_0(1); // straight

    // let the loco charge some before trying to move
    uint32_t track_on_us = time_us_32();
    const uint32_t charge_us = 5'000'000;
    const uint32_t delay_us = charge_us - (time_us_32() - track_on_us);
    loop(delay_us);

    while (true) {
        fetch();
        DesktopOps::uncouple(loco_id, loco);
        loop(2'000'000);
        spot();
        home();
    }

    sleep_ms(100);

    return 0;

} // main()


static void loop(int32_t for_us)
{
    int32_t end_us = int32_t(time_us_32()) + for_us;
    while (end_us - int32_t(time_us_32()) >= 0)
        SysLed::loop();

} // loop


// On entry:
// * Loco left of uncoupler
// * Car one loco-length to the right of the uncoupler
// On return:
// * Loco+car right of uncoupler, coupled
static void fetch()
{
    printf("fetch\n");

    if (Desktop::sensor_unc())
        printf("unexpected: uncoupler sensor active\n");

    // slow to uncoupler
    DccApi::loco_speed_set(loco_id, -loco->mms_to_dcc(DesktopOps::slow_mms));
    while (!Desktop::sensor_unc())
        loop();
    // rear of loco is at uncoupler
    DccApi::loco_speed_set(loco_id, -loco->mms_to_dcc(DesktopOps::creep_mms));
    while (Desktop::sensor_unc())
        loop();
    // front of loco has cleared uncoupler
    loop(mm_to_us(10, DesktopOps::creep_mms));
    DccApi::loco_speed_set(loco_id, DesktopOps::stop);
    loop(1'000'000);

    if (Desktop::sensor_unc())
        printf("unexpected: uncoupler sensor active\n");

    printf("fetch: done\n");
}


// On entry:
// * Loco should be left of uncoupler, coupler clear of magnet
// * Car should be right of uncoupler, with its coupler over the magnet
// On return:
// * Loco & car right of uncoupler, not coupled
static void spot()
{
    printf("spot\n");

    if (Desktop::sensor_unc())
        printf("unexpected: uncoupler sensor active\n");

    // creep back until rear of loco gets to sensor
    DccApi::loco_speed_set(loco_id, -loco->mms_to_dcc(DesktopOps::creep_mms));
    while (!Desktop::sensor_unc())
        loop();
    printf("spot: rear\n");

    // continue until loco clears uncoupler
    while (Desktop::sensor_unc())
        loop();
    printf("spot: clear\n");

    DccApi::loco_speed_set(loco_id, DesktopOps::stop);
    loop(1'000'000);

    if (Desktop::sensor_unc())
        printf("unexpected: uncoupler sensor active\n");

    printf("spot: done\n");
}


// On entry:
// * Loco & car right of uncoupler, not coupled
// On return:
// * Loco left of uncoupler, about half a car length past it
// * Car right of uncoupler, about a loco-lenth away
static void home()
{
    printf("home\n");

    DccApi::loco_speed_set(loco_id, loco->mms_to_dcc(DesktopOps::medium_mms));
    while (!Desktop::sensor_unc())
        loop();
    printf("home: nose\n");

    DccApi::loco_speed_set(loco_id, loco->mms_to_dcc(DesktopOps::slow_mms));
    while (Desktop::sensor_unc())
        loop();
    printf("home: rear\n");

    // go another 60 mm and stop
    int more_mm = 60 - loco->stop_mm(DesktopOps::slow_mms);
    if (more_mm > 0)
        loop(mm_to_us(more_mm, DesktopOps::slow_mms));
    printf("home: stop\n");

    DccApi::loco_speed_set(loco_id, DesktopOps::stop);
    loop(1'000'000);

    printf("home: done\n");
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
