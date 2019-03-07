#include "artl/digital_in.h"
#include "artl/digital_out.h"
#include "artl/pin_change_int.h"
#include "artl/yield.h"
#include "artl/tc.h"

using trigger = artl::digital_out<3>; // 10

enum {
    echo_pin = 4, //artl::pin::id::mini_13,  // or micro_15
};

using echo = artl::digital_in<echo_pin>;
using echo_int = artl::pin_change_int<echo::pin>;

using pump = artl::digital_out<0>; // 8

using input_tc = artl::tc<1>;

volatile unsigned long echo_on_us = 0;
volatile unsigned long echo_off_us = 0;
volatile bool echo_off_wait = false;

volatile bool enable_input = false;
unsigned long last_print = 0;

#define H_AVE_HWSERIAL 1

#if defined(ISR) && defined(PCINT0_vect)
ISR(PCINT0_vect) {
    if (echo().read()) {
        echo_on_us = micros();
    } else {
        echo_off_us = micros();
        echo_off_wait = false;
    }
}
#endif

#if defined(ISR) && defined(TIMER1_COMPA_vect)
ISR(TIMER1_COMPA_vect)
{
    enable_input = true;
}
#endif

void setup() {
#if defined(HAVE_HWSERIAL)
    Serial.begin(115200);
#endif

    trigger().setup();
    echo().setup();
    pump().setup();

    echo_int().enable();
    echo_int().disable();

#if defined(ARDUINO_ARCH_AVR)
    input_tc().setup(0, 0, 4, input_tc::cs::presc_256);
    input_tc().ocra() = 200;
    input_tc().cnt() = 0;
    input_tc().oca().enable();
#endif

    artl::yield();
}

bool pump_enabled = false;

bool tap_approach = false;
unsigned long tap_approach_start = 0;

const unsigned long min_tap_approach = 500; /* ms */
const double min_tap_distance = 15; /* cm */

void pump_on() {
    if (pump_enabled) {
        return;
    }

    pump_enabled = true;
    pump().high();

#if defined(HAVE_HWSERIAL)
    Serial.println("pump ON");
#endif
}

void pump_off() {
    if (!pump_enabled) {
        return;
    }

    pump_enabled = false;
    pump().low();

#if defined(HAVE_HWSERIAL)
    Serial.println("pump OFF");
#endif
}

void set_distance(double d) {
#if defined(HAVE_HWSERIAL)
//    Serial.println(d);
#endif

    if (d > min_tap_distance) {
        pump_off();
        if (tap_approach) {
            tap_approach = false;
#if defined(HAVE_HWSERIAL)
            Serial.println("tap distancing ...");
#endif
        }
    } else {
        if (tap_approach) {
            if (millis() - tap_approach_start >= min_tap_approach) {
                pump_on();
            }
        } else {
#if defined(HAVE_HWSERIAL)
            Serial.println("tap approach ...");
#endif
            tap_approach = true;
            tap_approach_start = millis();
        }
    }
}

void loop() {

    if (enable_input) {
        enable_input = false;

        echo_int().disable();

        if (echo_off_wait) {
            echo_off_wait = false;
            set_distance(1000);
        }

        trigger().low();
        delayMicroseconds(5);
        trigger().high();
        delayMicroseconds(10);
        trigger().low();

        echo_off_wait = true;
        echo_on_us = 0;
        echo_off_us = 0;
        echo_int().enable();

        artl::yield();
        return;
    }

    if (echo_off_us) {
        echo_int().disable();

        if (echo_on_us) {
            set_distance( (echo_off_us - echo_on_us) / 2 / 29.1 );

#if defined(DEBUG)
            if ((millis() - last_print) > 100) {
#if defined(HAVE_HWSERIAL)
/*
                Serial.print("off: ");
                Serial.print(echo_on_us);
                Serial.print(" - ");
                Serial.print(echo_off_us);
                Serial.print(" = ");
*/
                Serial.println( last_distance );
#endif
                last_print = millis();
            }
#endif
        } else {
            set_distance(999);
        }

        echo_off_us = 0;
    }

    artl::yield();
}
