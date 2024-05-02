#include "Arduino.h"
#include <string.h>

#include "LTCReader.h"
#include "TCDisplayController.h"

#define ICP1 8 // the LTC input pin // 8 for atmega368, 4 for atmega32u4 //
#define LTC_IN_PIN ICP1

#define SIGNAL_LED_PIN 13
#define LOCK_LED_PIN 6
#define HALL_SENSOR_PIN 2

#define FRAME_RATE 24 // mock for DEBUG -> TODO: detect this
#define DISPLAY_HOLD_ON_CLAP_MILLISEC 500

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// PROTOTYPES

void set_flags_for_LTC_reading_interrupt();
void set_flags_for_hall_sensor_interrupt();

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GLOBAL VARIABLES

LTCReader reader;

DFRobot_8DigSegDisplController tc_display_controller;

// hall sensor HIGH = clapper_is_open
volatile boolean clapper_is_open = false;
volatile boolean just_clapped = false;

// set to true in the clapper ISR, false when main loop detects true
volatile boolean should_turn_on_display = true;

// displayed upon clap
char* tc_at_clap = "--.--.--.--";

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// MAIN ARDUINO FUNCTIONS

void setup()
{
    Serial.begin(115200);
    Serial.println("PROGRAM RUNNING"); // DEBUG

    pinMode(LTC_IN_PIN, INPUT); // ICP pin (digital pin 8 on arduino) as input
    pinMode(SIGNAL_LED_PIN, OUTPUT);
    digitalWrite(SIGNAL_LED_PIN, LOW);
    pinMode(LOCK_LED_PIN, OUTPUT);
    digitalWrite(LOCK_LED_PIN, LOW);
    pinMode(HALL_SENSOR_PIN, INPUT);

    set_flags_for_hall_sensor_interrupt();
    set_flags_for_LTC_reading_interrupt();

    tc_display_controller.init_display();

    clapper_is_open = digitalRead(HALL_SENSOR_PIN);
    if (!clapper_is_open) {
        tc_display_controller.display_off();
    }
}

/* check for a new frame to print, print it (decoder) */
void loop()
{
    // indicate valid LTC signal -> set signal LED hi once we've counted 80 bits
    // (a complete LTC frame)
    digitalWrite(SIGNAL_LED_PIN, reader.is_reading_valid_frames() ? HIGH : LOW);
    // set sync lock indicator LED hi when synced
    digitalWrite(LOCK_LED_PIN, reader.get_state() == SYNC ? HIGH : LOW);

    if (reader.get_state() == NO_SYNC) {
        Serial.println("- NO SYNC -");
        return;
    }

    // STATE IS SYNC

    if (just_clapped) {
        just_clapped = false; // reset for next clap
        tc_display_controller.freeze_display(
            tc_at_clap, DISPLAY_HOLD_ON_CLAP_MILLISEC);
        tc_display_controller.freeze_display(
            reader.get_ub_string(), DISPLAY_HOLD_ON_CLAP_MILLISEC);
    }

    else if (clapper_is_open) {
        if (should_turn_on_display) {
            should_turn_on_display = false;
            tc_display_controller.display_on();
        }
        if (reader.is_new_frame) {
            tc_display_controller.print(reader.get_tc_string());
        }
    }

    else { // clapper has been closed for some time
        if (reader.is_new_second()) {
            tc_display_controller.flash_sync_indicator(FRAME_RATE);
        }
    }

    // leave if we've yet to see the end of this frame
    if (!reader.is_new_frame)
        return;

    // reached end of a frame, so there's new data to print

    reader.is_new_frame = false; // reset

    reader.decode_tc();
    reader.decode_ub();

    Serial.println("Frame: " + String(reader.running_frame_count - 1) + " - " + reader.get_tc_string());
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// INTERRUPT ROUTINES

/* triggered when a transition (?) is detected at the input capture pin */
#define INTERRUPT_ROUTINE_tc_reader() ISR(TIMER1_CAPT_vect)

INTERRUPT_ROUTINE_tc_reader()
{
    /*
    Toggle edge capture. ICES1 = input capture edge select. Specifies whether
    this capture should happen with the rising edge (ICES1 = 1) or the falling
    edge (ICES1 = 0).

    WHY?
    According to the LTC modulation scheme, each bit period starts at the
    opposite state of the last. So if this bit starts hi, next bit starts low.
    This also accounts for the mid-bit-period transition that defines a binary
    1. Apparently with each transition/firing of this ISR, we need to start
    listening for the opposite transition.
    */
    TCCR1B ^= _BV(ICES1);
    TCNT1 = 0; // reset counter

    reader.update(ICR1);
}

/* Triggered upon signal loss (or discontinuity?) at input capture pin.
Discontinuity detection happens via the counter overflow. The counter gets reset
periodically durring SYNC state. */
#define INTERRUPT_ROUTINE_tc_reader_signal_loss() ISR(TIMER1_OVF_vect)

INTERRUPT_ROUTINE_tc_reader_signal_loss()
{
    reader.reset();
}

/* Triggered by state change of hall sensor on pin D2, i.e. on clap or open
clapper. */
#define tc_reader_clapper_change_interrupt_routine() ISR(PCINT2_vect)

tc_reader_clapper_change_interrupt_routine()
{
    noInterrupts();

    clapper_is_open = digitalRead(HALL_SENSOR_PIN);

    if (clapper_is_open) {
        should_turn_on_display = true;
        Serial.println("OPEN CLAPPER");
    } else {
        just_clapped = true;
        strcpy(tc_at_clap, reader.get_tc_string()); // capture the timecode at the clap
        Serial.println("CLOSE CLAPPER");
    }

    interrupts();
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// HELPERS

void set_flags_for_LTC_reading_interrupt()
{
    noInterrupts();

    TCCR1A = B00000000; // clear all
    TCCR1B = B11000010; // ICNC1 noise reduction + ICES1 start on rising edge + CS11 divide by 8
    TCCR1C = B00000000; // clear all
    TIMSK1 = B00100001; // ICIE1 (bit 5) enable the icp, and TOIE1 (bit 0) for the overflow
    TCNT1 = 0; // clear timer1

    interrupts();
}

/*
Listen on pin D2 for state change (of hall sensor) to trigger an interrupt
(IOW, set up a [P]in [C]hange [I]nterrupt)
PCICR = pin change interrupt control register
PCMSK2 = pin change interrupt mask, group 2)
*/
void set_flags_for_hall_sensor_interrupt()
{
    // listen for state change on interrupt group 2 (port D) pins
    PCICR |= B00000100;
    // listen only on pin D2 (aka pin2 grp2) for state changes
    PCMSK2 |= B00000100;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// HANDLERS

/* TODO:
- redo the state machine
- debounce hall sensor
- implement:
    // determines the framerate and informs the user via the segmented display
    void report_framerate()
- bugfixes:
    - display never re-initializes if its input stream is interrupted
        - how would a display input interruption be detected?
*/