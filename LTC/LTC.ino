#include "Arduino.h"
#include "DFRobot_LedDisplayModule.h"
#include "DecodedLTC.h"
#include "LTCFrame.h"
#include "LTCGenerator.h"
#include "TCDisplayController.h"

#include <string.h>

#define LTC_IN ICP1
#define ICP1 8 // ICP1, 8 for atmega368, 4 for atmega32u4
#define LTC_OUT 9
#define SIGNAL_LED 13
#define LOCK_LED 6

/* TODO: how are these BIT_TIME_X constants used?
    seems to be an error check
        if the width in microsec of a decoded bit is too short or long to be
            considered an LTC bit ???
    if this was the case, wouldn't we have to measure the time between
        hi/lo transitions somewhere?
            is that happening somewhere?
 */
#define BIT_TIME_THRESHOLD 700
#define BIT_TIME_MIN 250
#define BIT_TIME_MAX 1500

#define FRAME_RATE 24 // mock for DEBUG -> TODO: detect this
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// PROTOTYPES

void decode_and_update_time_vals(DecodedLTC*, byte*);
void decode_UB_and_update_vals(DecodedLTC*, byte*);
void update_TC_string();

void startLTCDecoder();
// void stopLTCDecoder();
// void startLTCGenerator();

void setup_hall_sensor_for_pin_change_interrupt();
void handle_clap();

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GLOBAL VARIABLES

TcDisplayController tc_display_controller;

// the timecode string to print: 8 digits + 3 separators (.) + \0
// this format (with periods) is required by the DFRobot segment display lib
char* TC_string = "00.00.00.00";

// the user bits string to print: same format as SMPTE_string
char* UB_string = "UB.UB.UB.UB";

DecodedLTC decoded;

// store 2 frames -- TODO: WHY 2? why not initialize to all zeros?
/*
NOTE: each frame is 10 bytes or 80 bits (the LTC standard). The last two bytes
of each frame is the sync pattern.
*/
volatile LTCFrame frames[2] = {
    { 0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF },
    { 0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF }
};
volatile byte currentFrameIndex; // index of frames[2] -- can be 0 or 1 (ASSUMPTION)

volatile boolean frameAvailable; // flag - indicates received last bit of an LTC frame

/*
The LTC spec's sync word: fixed bit pattern 0011 1111 1111 1101.
Used to detect the end of a frame.
*/
const unsigned short SYNC_PATTERN = 0xBFFC;
// read from incoming LTC
// when matches SYNC_PATTERN, indicates end of a frame (frameAvailable = true)
volatile unsigned short syncValue;

/*
A misnomer - there is no clock to speak of. Refers to mutually
exclusive states related to reading and generating an LTC signal.
*/
enum ClockState {
    NO_SYNC,
    SYNC,
    // GENERATE // NO GENERATING YET!
};
volatile unsigned int clockState = ClockState::NO_SYNC;

volatile unsigned long validFrameCount; // running counter of valid frames decoded
volatile unsigned short validBitCount; //            " "            bits decoded
// counts bits up to 80, resets upon frameAvailable (got last bit of a frame)
volatile byte frameBitCount;

volatile byte oneFlag = 0;
// volatile boolean curr_bit_val;
// volatile boolean last_bit_val;
volatile unsigned int bitTime; // TODO: is this the width of the LTC bit in time? as in, 1sec / frame_rate / LTC's_80bits

// used to update ltc data in the ICP1 interrupt (LTC input) (decode mode)
byte idx, bIdx;

volatile byte* current_frame_bytes; // index into this to access the bytes of the current LTC frame (decode mode)

// int previousOutputFrameIndex = 0 // HAS NO REFS?

// hall effect sensor macros and globals
#define HALL_SENSOR_PIN 2
#define DISPLAY_HOLD_MILLISEC 500
volatile boolean just_clapped = false; // true -> display tc_on_clap
volatile boolean clapper_is_open = false; // true -> display on
/* the string that's displayed upon clap - a copy of the SMPTE_string at clap.
also displayed upon boot up to indicate boot status. */
char* tc_on_clap = "--.--.--.--";
char* UB_on_clap = "UB.UB.UB.UB";

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// MAIN ARDUINO FUNCTIONS

void setup()
{
    Serial.begin(115200);
    Serial.println("PROGRAM RUNNING"); // DEBUG

    pinMode(LTC_IN, INPUT); // ICP pin (digital pin 8 on arduino) as input
    pinMode(LTC_OUT, OUTPUT);

    pinMode(SIGNAL_LED, OUTPUT);
    pinMode(LOCK_LED, OUTPUT);

    digitalWrite(SIGNAL_LED, LOW);
    digitalWrite(LOCK_LED, LOW);

    setup_hall_sensor_for_pin_change_interrupt();

    tc_display_controller.init_display();

    // hall sensor HIGH means clapper_is_open
    clapper_is_open = digitalRead(HALL_SENSOR_PIN);
    if (!clapper_is_open) {
        tc_display_controller.display_off(); // TODO: can't i do this once instead of every iter?
    }

    // set the mode of operation
    startLTCDecoder();
    // startLTCGenerator();
}

/* check for a new frame to print, print it (decoder) */
void loop()
{
    // indicate valid LTC signal -> set signal LED hi once we've counted 80 bits
    // (a complete LTC frame)
    digitalWrite(SIGNAL_LED, validBitCount > 80 ? HIGH : LOW);
    // set sync lock indicator LED hi when synced
    digitalWrite(LOCK_LED, clockState == ClockState::SYNC ? HIGH : LOW);

    // // LEAVE ME COMMENTED OUT -- NO GENERATOR YET
    // if (clockState == ClockState::GENERATE) {
    //     generator.update();
    //     return;
    // }

    if (clockState == ClockState::NO_SYNC) {
        Serial.println("STATE IS NO_SYNC. HANDLE THIS CASE.");
        return;
    }

    // STATE IS SYNC

    // clapper handling (via hall effect sensor)
    if (just_clapped) {
        handle_clap();
    }

    else if (clapper_is_open) {
        handle_open_clapper();
    }

    else {
        if (decoded.f == 0) {
            // Serial.println("SECOND MARK");
            // TODO: the following does not work!
            tc_display_controller.flash_sync_indicator(FRAME_RATE);
        }
    }

    // only proceed to update time vars if we've past the end of an LTC frame
    if (!frameAvailable)
        return;

    frameAvailable = false;

    // reached end of a frame, so there's new data to print

    current_frame_bytes = frames[1 - currentFrameIndex];

    Serial.print("Frame: ");
    Serial.print(validFrameCount - 1);
    Serial.print(" - ");

    // decode time values and update the timecode string to be displayed
    decode_and_update_time_vals(&decoded, current_frame_bytes);
    update_TC_string();

    // update the user bits string (but don't display it until appropriate)
    decode_UB_and_update_vals(&decoded, current_frame_bytes);
    update_UB_string();
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// DECODER

volatile boolean curr_bit_val, last_bit_val;

#define LAST_LTC_BIT_INDEX 79
/* triggered when a transition (?) is detected at the input capture pin */
ISR(TIMER1_CAPT_vect)
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
    bitTime = ICR1; // store counter value at edge / ICR = input capture register
    TCNT1 = 0; // reset counter

    // remove out of range value // TODO: make this comment better
    if (bitTime < BIT_TIME_MIN || bitTime > BIT_TIME_MAX) {
        // reset everything
        syncValue = 0;
        validBitCount = 0;
        validFrameCount = 0;
        frameAvailable = false;
        clockState = ClockState::NO_SYNC;
        return;
    }

    // increment valid bit counts, without overflow
    validBitCount = validBitCount < 65535
        ? validBitCount + 1
        : 0;

    /*
    In the LTC modulation scheme, "1" bit has 2 transitions per bit period where
    a "0" bit has 1 transition. If the measured period between transitions is
    greater than that required to make a 1 bit, it must be a 0 bit.
    */
    curr_bit_val = bitTime > BIT_TIME_THRESHOLD
        ? 0
        : 1;

    // don't count 1 twice! TODO: What does this do? DUNNO but it breaks without it!
    if (curr_bit_val == 1 && last_bit_val == 1) {
        last_bit_val = 0;
        return;
    }

    last_bit_val = curr_bit_val;

    // update frame sync pattern detection
    syncValue = (syncValue >> 1) + (curr_bit_val << 15);

    // update clockState
    switch (clockState) {
    case ClockState::NO_SYNC:
        // sync pattern detected
        if (syncValue == SYNC_PATTERN) {
            clockState = ClockState::SYNC;
            frameBitCount = 0;
            return;
        }
        break;
    case ClockState::SYNC:
        // if it seems we reached the end of a frame but didn't see the sync pattern,
        // or if we see a sync pattern but have not counted
        if ((frameBitCount > LAST_LTC_BIT_INDEX && syncValue != SYNC_PATTERN)
            /* || (syncValue == SYNC_PATTERN && frameBitCount != LAST_LTC_BIT_INDEX) */) {
            // something went wrong!
            syncValue = 0;
            validBitCount = 0;
            validFrameCount = 0;
            frameAvailable = false;
            clockState = ClockState::NO_SYNC;
            return;
        }

        // if this is the last bit of a frame
        if (syncValue == SYNC_PATTERN) {
            frameAvailable = true; // signal that we've captured a full frame

            // reset bit counter and increment total frame count
            frameBitCount = 0;
            validFrameCount++;

            // alternates between 0 and 1: if 1, becomes 0; if 0, becomes 1
            currentFrameIndex = 1 - currentFrameIndex;

            return;
        }

        // we're on a bit between the start & end of the current frame

        // ptr to the current frame in the array of LTC frames (doesn't compile if global)
        volatile byte* f = frames[currentFrameIndex]; // (grab all the bits we've gathered for this frame so far)

        // TODO: why is this different from currentFrameIndex?
        idx = frameBitCount / 8; // get the index of a byte from frames[]
        bIdx = frameBitCount & 0x07; // the bit index of this frame

        // update the current bit (in its byte at f[idx]) with the value of curr_bit_val
        f[idx] = (f[idx] & ~(1 << bIdx)) | (curr_bit_val << bIdx);

        /*
        if (curr_bit_val)
            f[idx] |= 1 << bIdx;
        else
            f[idx] &= ~(1 << bIdx);
        */

        frameBitCount++;
        return;
    }
}

/* triggered upon signal loss (or discontinuity?) at input capture pin */
ISR(TIMER1_OVF_vect)
{
    // NOT GENERATING YET!/
    // if (clockState == ClockState::GENERATE)
    //     return;

    // if we overflow, we then lost signal
    syncValue = 0;
    validBitCount = 0;
    validFrameCount = 0;
    frameAvailable = false;
    clockState = ClockState::NO_SYNC;
}

/* triggered by state change of hall sensor on pin D2 */
ISR(PCINT2_vect)
{
    noInterrupts();

    // interrupt was triggered by OPENING the clapper
    // (clapper STATE currently CLOSED -- must be closed to open)
    if (!clapper_is_open) {
        clapper_is_open = true;
        Serial.println("OPEN CLAPPER");
    }
    // interrupt was triggered by CLOSING the clapper
    // (clapper STATE currently OPEN -- must be open to close)
    else {
        clapper_is_open = false;
        just_clapped = true;
        Serial.println("CLOSE CLAPPER");
    }

    interrupts();
}

void startLTCDecoder()
{
    noInterrupts();
    TCCR1A = B00000000; // clear all
    TCCR1B = B11000010; // ICNC1 noise reduction + ICES1 start on rising edge + CS11 divide by 8
    TCCR1C = B00000000; // clear all
    TIMSK1 = B00100001; // ICIE1 (bit 5) enable the icp, and TOIE1 (bit 0) for the overflow

    TCNT1 = 0; // clear timer1

    validBitCount = 0;
    bitTime = 0;
    syncValue = 0;
    currentFrameIndex = 0;
    validFrameCount = 0;
    frameAvailable = false;
    clockState = ClockState::NO_SYNC;
    interrupts();
}

void stopLTCDecoder()
{
    noInterrupts();
    TIMSK1 &= ~(1 << ICIE1);
    TIMSK1 &= ~(1 << TOIE1);

    TCCR1B &= ~(1 << CS12);
    TCCR1B &= ~(1 << CS11);
    TCCR1B &= ~(1 << CS10);
    interrupts();

    digitalWrite(SIGNAL_LED, LOW); // valid after 1 frame
    digitalWrite(LOCK_LED, LOW);

    frameAvailable = false;
    clockState = ClockState::NO_SYNC;
}

/*
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GENERATOR

ISR(TIMER1_COMPA_vect)
{
    generator.interupt();
}

void startLTCGenerator()
{
    noInterrupts();
    TCCR1A = 0; // clear all
    TCCR1B = (1 << WGM12) | (1 << CS10);
    TIMSK1 = (1 << OCIE1A);

    TCNT1 = 0;

    // 30 fps, 80 bits * 30
    OCR1A = 3333; // = 16000000 / (30 * 80 * 2)

    state = ClockState::GENERATE;
    generator.reset();
    interrupts();
}
 */

/*
Listen on pin D2 for state change (of hall sensor) to trigger an interrupt
(IOW, set up a [P]in [C]hange [I]nterrupt)
PCICR = pin change interrupt control register
PCMSK2 = pin change interrupt mask, group 2)
*/
void setup_hall_sensor_for_pin_change_interrupt()
{
    // listen for state change on interrupt group 2 (port D) pins
    PCICR |= B00000100;
    // listen only on pin D2 (aka pin2 grp2) for state changes
    PCMSK2 |= B00000100;
    pinMode(HALL_SENSOR_PIN, INPUT);
}

/* Update the timecode string before printing */
void update_TC_string()
{
    sprintf(
        TC_string,
        "%02d.%02d.%02d.%02d",
        decoded.h, decoded.m, decoded.s, decoded.f);
}

/* Update the user bits string before printing */
void update_UB_string()
{
    sprintf(
        UB_string,
        "%d%d.%d%d.%d%d.%d%d",
        decoded.ub0, decoded.ub1, decoded.ub2, decoded.ub3, decoded.ub4, decoded.ub5, decoded.ub6, decoded.ub7);
}

/*
Decode Binary Coded Decimal (BCD) values from specific bits of an LTC frame
and store the values in the decodedLTC struct.
*/
void decode_and_update_time_vals(DecodedLTC* decodedLTC, volatile byte* curr_frame_bytes)
{
    // 10s place + 1s place
    decodedLTC->h = (curr_frame_bytes[7] & 0x03) * 10 + (curr_frame_bytes[6] & 0x0F); // 0x03 -> smallest 2 bits (2+1=3)
    decodedLTC->m = (curr_frame_bytes[5] & 0x07) * 10 + (curr_frame_bytes[4] & 0x0F); // 0x07 -> smallest 3 bits (4+2+1=7)
    decodedLTC->s = (curr_frame_bytes[3] & 0x07) * 10 + (curr_frame_bytes[2] & 0x0F); // 0x0F -> smallest 4 bits (8+4+2+1=F)
    decodedLTC->f = (curr_frame_bytes[1] & 0x03) * 10 + (curr_frame_bytes[0] & 0x0F);

    /* Explanation for Dummies

        - least significant bit first (small half is left half, large half is right half if reading chronologically)
            - e.g. - frame number comes from the 0th and 1st bytes of the current frame,
            hour from the 6th and 7th

        - possible frame rates: 23.976, 24, 25, 29.97, 30

        - max value representable with 4 bits, 3 bits, 2bits
          1 1 1 1 = 15,  1 1 1 = 7,  1 1 = 3

        field/char #     small 1/2 of    LTC bits            unit            max value needed
        ++++++++++++     ++++++++++++    ++++++++            ++++            +++++++
                   8     byte 0          00-03 (4 bits)      frame 1's       9 (as in 29 frames)
                   7     byte 1          08-09 (2 bits)      frame 10's      3 (as in 30 frames)
                   6     byte 2          16-19 (4 bits)      seconds 1's     9 (as in 59 seconds)
                   5     byte 3          24-26 (3 bits)      seconds 10's    5 (as in 59 seconds)
                   4     byte 4          32-35 (4 bits)      minutes 1's     9 (as in 59 minutes)
                   3     byte 5          40-42 (3 bits)      minutes 10's    5 (as in 59 minutes)
                   2     byte 6          48-51 (4 bits)      hours 1's       3 (as in 23 hours)
                   1     byte 7          56-57 (2 bits)      hours 10's      2 (as in 23 hours)

        Example:
            - current hour is 12
            - 7th index is hour 10's place, 6th is 1's place
                - 7th index = 1, 6th = 2
                - small 1/2 of 7th and 6th bytes of 10 byte (80 bit) long frame

                    7th byte (2 least sig bits) - representing hour 10's place (least to most significant bit):
                    [ 0 0 0 1 ]-0-0-0-0  ->  & 0x03  ->
                    & 0 0 1 1
                    = 0 0 0 1  = decimal value 1

                    6th byte (4 least sig bits) - representing hour 1's place (least to most significant bit):
                    [ 0 0 1 0 ]-0-0-0-0  -> & 0x0F  ->
                    & 1 1 1 1
                    = 0 0 1 0  = decimal value 2

                    What have we done here?
                        - decoded decimal values from binary values

    */
}

/* from the 8 data bytes of an LTC frame (remaining 2 for sync word), decode
user bit hex vals and store them in their respective globals */
void decode_UB_and_update_vals(DecodedLTC* decodedLTC, volatile byte* curr_frame_bytes)
{
    /* rshift 4 bc, despite picking out the large half of LTC byte, we're
    using 1,2,4,8 places only, as that's all that's needed to make a hex char
    - (right shift is always towards LSB)
    - each ubX is 1 digit bt 0 and F (aka a hex char)
    - ...& 0xF0 -> access largest 4 bits of this LTC byte by masking off the
        smallest 4 bits */
    decodedLTC->ub7 = (curr_frame_bytes[0] & 0xF0) >> 4;
    decodedLTC->ub6 = (curr_frame_bytes[1] & 0xF0) >> 4;
    decodedLTC->ub5 = (curr_frame_bytes[2] & 0xF0) >> 4;
    decodedLTC->ub4 = (curr_frame_bytes[3] & 0xF0) >> 4;
    decodedLTC->ub3 = (curr_frame_bytes[4] & 0xF0) >> 4;
    decodedLTC->ub2 = (curr_frame_bytes[5] & 0xF0) >> 4;
    decodedLTC->ub1 = (curr_frame_bytes[6] & 0xF0) >> 4;
    decodedLTC->ub0 = (curr_frame_bytes[7] & 0xF0) >> 4;

    /* USER BITS BREAKDOWN

    32 bits are assigned as eight groups/fields/chars of four [USER] BITS (UB)

    32 bits = 4 bytes
    4 bytes / 8 fields = 1/2 byte per field...
    ... = 1x 4-bit binary number per field...

    4 bits allows for a single hexadecimal character per field / UB char (0-F),
    which is the standard format of UB in American film production

    e.g. encode the date and reel (to understand decoding)
        2023 Aug 7
        sound reel #3F

    GOAL USER BITS (UB)   23 : 08 : 07 : 3F -->

          2        3 :      0        8 :      0        7 :      3        F  <- UB goal / hex value
    0 0 1 0  0 0 1 1  0 0 0 0  1 0 0 0  0 0 0 0  0 1 1 1  0 0 1 1  0 1 1 0  <- encoded LTC stream
    - - - -  - - - -  - - - -  - - - -  - - - -  - - - -  - - - -  - - - -
    8 4 2 1  8 4 2 1  8 4 2 1  8 4 2 1  8 4 2 1  8 4 2 1  8 4 2 1  8 4 2 1  <- place - (may be backwards, too tired to tell atm)
          7        6        5        4        3        2        1        0  <- LTC byte number (UB lives in largest half of the byte)
          7        6        5        4        3        2        1        0  <- UB field/character (offset by 1 from spec, so we can count from 0)


    where are the user bit fields within the LTC bit array / schema?

        field/char #     large 1/2 of    LTC bits
        ++++++++++++     ++++++++++++    ++++++++
                   1     byte 0          04-07
                   2     byte 1          12-15
                   3     byte 2          20-23
                   4     byte 3          28-31
                   5     byte 4          36-39
                   6     byte 5          44-47
                   7     byte 6          52-55
                   8     byte 7          60-63

    how do we access the bits in the 2nd half of the data bytes 1-8
        (9-10 is the sync word)

        mask off the discarded bits (holding timecode)
            ... & 0xF0
            reveals position of any binary 1s in the larger half of the LTC byte

        right shift >> 4 places bc we're taking the larger half of a byte
            this would normally be the places 16, 32, 64, 128, but the LTC spec
            uses these 4 bits as places 1, 2, 4, 8 as that's all that's required
            to represent a hexadecimal character
    */
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// HANDLERS

void handle_clap()
{
    just_clapped = false; // reset for next clap
    tc_display_controller.freeze_display(tc_on_clap, UB_string);
}

void handle_open_clapper()
{
    tc_display_controller.display_on();
    tc_display_controller.print(TC_string);
}

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