#include "Arduino.h"
#include <string.h>

// #include "State.h"

// #include "EventData.h"

// #include "EventCallbacks.h"

// #include "Dispatcher.h"
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

enum DisplayInstruction : uint8_t /* 1 byte guarantees an atomic operation getting or setting. My theory is, keeping these ops atomic will prevent a race condition between the main loop and the reader ISR, which is triggering around 2000(?) times per second. */ {
    NONE = 10,
    TURN_OFF,
    TURN_ON,
    UPDATE,
    INDICATE_SYNC, // when clapper is close / display is "off"
};

volatile DisplayInstruction display_instruction = NONE;

// Dispatcher dispatcher = Dispatcher::get();

LTCReader reader;

DFRobot_8DigSegDisplController tc_display_controller;

// hall sensor HIGH = clapper_is_open
volatile boolean clapper_is_open = false;
volatile boolean just_clapped = false;

// set to true in the clapper ISR, false when main loop detects true
volatile boolean display_on = true;

// displayed upon clap
char* tc_at_clap = "--.--.--.--";

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// MAIN ARDUINO FUNCTIONS
void TEST_HANDLER()
{
    Serial.println("* * * * * * EVENT HANDLED! * * * * * *");
}

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

    tc_display_controller.init_display();

    noInterrupts();

    set_flags_for_hall_sensor_interrupt();

    clapper_is_open = digitalRead(HALL_SENSOR_PIN);
    if (!clapper_is_open) {
        tc_display_controller.display_off();
        // display_instruction = TURN_OFF;
    }

    // EVENT SUBSCRIPTIONS
    // dispatcher.subscribe(CLOSED_CLAPPER, TEST_HANDLER);

    set_flags_for_LTC_reading_interrupt(); // LAST LINE OF SETUP!

    interrupts();
}

#define TC_CHARS_LEN 11

#define DEBUG(MSG) Serial.println(MSG);

void handle_display_instruction(volatile DisplayInstruction& instruction, char tc_chars[TC_CHARS_LEN])
{
    switch (instruction) {
    case UPDATE:
        instruction = NONE;
        // DEBUG("HIT CASE UPDATE")
        tc_display_controller.print(tc_chars);
        // tc_display_controller.print("");
        break;
    case TURN_OFF:
        // DEBUG("HIT CASE TURN OFF")
        tc_display_controller.display_off();
        break;
    case TURN_ON:
        instruction = NONE;
        // DEBUG("HIT CASE TURN ON")
        tc_display_controller.display_on();
        break;
    case INDICATE_SYNC:
        instruction = NONE;
        // DEBUG("HIT CASE INDICATE SYNC")
        tc_display_controller.flash_sync_indicator();
        break;
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

    if (reader.is_new_frame) {
        display_instruction = UPDATE;
    }

    if (!clapper_is_open && reader.is_new_second()) {
        // tc_display_controller.flash_sync_indicator();
        display_instruction = INDICATE_SYNC;
    }

    handle_display_instruction(
        display_instruction,
        display_instruction == UPDATE ? reader.get_tc_string() : NULL);

    // if (just_clapped) {
    //     just_clapped = false; // reset for next clap
    //     tc_display_controller.freeze_display(
    //         tc_at_clap, DISPLAY_HOLD_ON_CLAP_MILLISEC);
    //     tc_display_controller.freeze_display(
    //         reader.get_ub_string(), DISPLAY_HOLD_ON_CLAP_MILLISEC);
    // }

    /*  else if (clapper_is_open) {

         if (display_on) {
             display_on = false;
             tc_display_controller.display_on();
         } else {
             tc_display_controller.display_off();
         }

         if (reader.is_new_frame) {
             tc_display_controller.print(reader.get_tc_string());
         }
         // if (reader.is_new_second()) {
         //     Serial.println(reader.get_tc_string());
         // }
     }

     else { // clapper has been closed for some time
         if (reader.is_new_second()) {
             tc_display_controller.flash_sync_indicator(FRAME_RATE);
         }
     } */

    // leave if we've yet to see the end of this frame
    if (!reader.is_new_frame)
        return;

    // reached end of a frame, so there's new data to print

    reader.is_new_frame = false; // reset

    reader.decode_tc();
    reader.decode_ub();

    // Serial.println("Frame: " + String(reader.running_frame_count - 1) + " - " + reader.get_tc_string());
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// INTERRUPT ROUTINES

/* triggered when a transition (?) is detected at the input capture pin */
#define INTERRUPT_ROUTINE_tc_reader() ISR(TIMER1_CAPT_vect)

INTERRUPT_ROUTINE_tc_reader()
{
    // noInterrupts();
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

    // interrupts();
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
        Serial.println("OPENED CLAPPER");
        // display_on = true;
        display_instruction = TURN_ON;
    } else {
        Serial.println("CLOSED CLAPPER");
        // display_on = false;
        display_instruction = TURN_OFF;

        // just_clapped = true;

        // TODO: is this really necessary? won't it be fast enough to get it in
        // the main loop?
        // strcpy(tc_at_clap, reader.get_tc_string()); // capture the timecode at the clap
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