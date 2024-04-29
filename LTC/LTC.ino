#include "Arduino.h"
#include "DFRobot_LedDisplayModule.h"
#include "LTCFrame.h"
#include "LTCGenerator.h"

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

// decoding LTC stream + updating vars
void decode_and_update_time_vals(volatile byte*);
void decode_UB_and_update_vals();
void update_TC_string();

// segmented display
void wait_for_display();
void print_to_segment_display(char* SMPTE_string);
void flash_sync_indicator();

// main modes of the machine
void startLTCDecoder();
void stopLTCDecoder();
void startLTCGenerator();

// hall effect sensor
void setup_hall_sensor_for_pin_change_interrupt();
void freeze_display();
void handle_clap();

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GLOBAL VARIABLES

// instantiate segmented display object with the IIC address of the display
DFRobot_LedDisplayModule LED(&Wire, 0xE0);

// the timecode string to print: 8 digits + 3 separators (.) + \0
// this format (with periods) is required by the DFRobot segment display lib
char* TC_string = "00.00.00.00";

// the user bits string to print: same format as SMPTE_string
char* UB_string = "UB.UB.UB.UB";

// the time values (in decimal) extracted from LTC in the decoder
typedef struct DecodedLTC {
    uint8_t h, m, s, f;
    uint8_t ub7, ub6, ub5, ub4, ub3, ub2, ub1, ub0;
};
DecodedLTC decoded;

// store 2 frames -- TODO: WHY 2? why not initialize to all zeros?
volatile LTCFrame frames[2] = {
    { 0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF },
    { 0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF }
};
volatile byte currentFrameIndex; // index of frames[2] -- can be 0 or 1 (ASSUMPTION)

volatile boolean frameAvailable; // indicates received last bit of an LTC frame

// the LTC spec's sync word: fixed bit pattern 0011 1111 1111 1101
const unsigned short SYNC_PATTERN = 0xBFFC;
// read from incoming LTC
// when matches SYNC_PATTERN, indicates end of a frame (frameAvailable = true)
volatile unsigned short syncValue;

// states of the machine
#define NOSYNC 0
#define SYNCED 1
#define GENERATOR 2
volatile char state = NOSYNC;

// running counter of valid frames decoded
volatile unsigned long validFrameCount;
volatile unsigned short validBitCount;
// counts bits up to 80, resets upon frameAvailable (got last bit of a frame)
volatile byte frameBitCount;

volatile byte oneFlag = 0;
volatile byte currentBit;
volatile byte lastBit;
volatile unsigned int bitTime; // TODO: is this the width of the LTC bit in time? as in, 1sec / frame_rate / LTC's_80bits

// used to update ltc data in the ICP1 interrupt (LTC input) (decode mode)
byte idx, bIdx;

volatile byte* current_frame_bytes; // index into this to access the bytes of the current LTC frame (decode mode)

int previousOutputFrameIndex = 0;

// hall effect sensor macros and globals
#define HALL_SENSOR_PIN 2
#define DISPLAY_HOLD_MILLISEC 500
volatile boolean just_clapped = false; // true -> display TC_on_clap
volatile boolean clapper_is_open; // true -> display on
/* the string that's displayed upon clap - a copy of the SMPTE_string at clap.
also displayed upon boot up to indicate boot status. */
char TC_on_clap[12] = {
    '-', '-', '.', '-', '-', '.', '-', '-', '.', '-', '-', '\0'
};
char UB_on_clap[12] = {
    'U', 'B', '.', 'U', 'B', '.', 'U', 'B', '.', 'U', 'B', '\0'
};

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

    wait_for_display();

    // enable all 8 digits on the display
    LED.setDisplayArea(1, 2, 3, 4, 5, 6, 7, 8);

    // indicate that the system is booting up by displaying dashes and dots
    print_to_segment_display(TC_on_clap);
    delay(DISPLAY_HOLD_MILLISEC);

    // hall sensor HIGH means clapper_is_open
    clapper_is_open = digitalRead(HALL_SENSOR_PIN);
    if (!clapper_is_open) {
        LED.displayOff();
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
    digitalWrite(LOCK_LED, state == SYNCED ? HIGH : LOW);

    // // LEAVE ME COMMENTED OUT -- NO GENERATOR YET
    // if (state == GENERATOR) {
    //     generator.update();
    //     return;
    // }

    // mode of operation is DECODER, state is NOSYNC or SYNCED

    // clapper handling (via hall effect sensor)
    if (just_clapped)
        handle_clap();

    if (clapper_is_open) {
        handle_open_clapper();
    } else {
        if (decoded.f == 0) {
            // Serial.println("SECOND MARK");
            // TODO: the following does not work!
            flash_sync_indicator();
        }
    }

    // only proceed to update time vars if we've past the end of an LTC frame
    if (!frameAvailable)
        return;

    // reached end of a frame, so there's new data to print

    current_frame_bytes = frames[1 - currentFrameIndex];

    Serial.print("Frame: ");
    Serial.print(validFrameCount - 1);
    Serial.print(" - ");

    // decode time values and update the timecode string to be displayed
    decode_and_update_time_vals(current_frame_bytes);
    update_TC_string();

    // update the user bits string (but don't display it until appropriate)
    decode_UB_and_update_vals();
    update_UB_string();

    // reset
    frameAvailable = false;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// DECODER

/* triggered when a transition (?) is detected at the input capture pin */
ISR(TIMER1_CAPT_vect)
{
    TCCR1B ^= _BV(ICES1); // toggle edge capture / ICES1 = input capture edge select / specifies whether this capture should happen with the rising edge (when ICES1 = 1) or the falling edge (when ICES1 = 0).
    bitTime = ICR1; // store counter value at edge / ICR = input capture register
    TCNT1 = 0; // reset counter

    // remove out of range value
    if ((bitTime < BIT_TIME_MIN) || (bitTime > BIT_TIME_MAX)) {
        // reset everything
        syncValue = 0;
        validBitCount = 0;
        validFrameCount = 0;
        frameAvailable = false;
        state = NOSYNC;
        return;
    }

    // increment valid bit counts, without overflow
    validBitCount = validBitCount < 65535
        ? validBitCount + 1
        : 0;

    /* TODO: what are bitTime, BIT_TIME_THRESHOLD, and what's happening here?
    LTC 1 bit has 2 transitions per bit period where a 0 bit has 1 transition,
    so it would make sense that the bit we're reading now should be called a 0
    if there was a larger time between transitions hi/lo/hi

    is that what this is getting at?
    */
    currentBit = bitTime > BIT_TIME_THRESHOLD
        ? 0
        : 1;

    // don't count 1 twice!
    if (currentBit == 1 && lastBit == 1) {
        lastBit = 0;
        return;
    }
    lastBit = currentBit;

    // update frame sync pattern detection
    syncValue = (syncValue >> 1) + (currentBit << 15);

    // update state
    switch (state) {
    case NOSYNC:
        // sync pattern detected
        if (syncValue == SYNC_PATTERN) {
            state = SYNCED;
            frameBitCount = 0;
            return;
        }
        break;
    case SYNCED:
        if ((frameBitCount > 79 && syncValue != SYNC_PATTERN)
            || (syncValue == SYNC_PATTERN && frameBitCount != 79)) {
            // something went wrong!
            syncValue = 0;
            validBitCount = 0;
            validFrameCount = 0;
            frameAvailable = false;
            state = NOSYNC;
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
        byte* f = frames[currentFrameIndex]; // (grab all the bits we've gathered for this frame so far)

        // TODO: why is this different from currentFrameIndex?
        idx = frameBitCount / 8; // get the index of a byte from frames[]
        bIdx = frameBitCount & 0x07; // the bit index of this frame

        // update the current bit (in its byte at f[idx]) with the value of currentBit
        f[idx] = (f[idx] & ~(1 << bIdx)) | (currentBit << bIdx);

        /*
        if (currentBit)
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
    if (state == GENERATOR)
        return;

    // if we overflow, we then lost signal
    syncValue = 0;
    validBitCount = 0;
    validFrameCount = 0;
    frameAvailable = false;
    state = NOSYNC;
}

/* triggered by state change of hall sensor on pin D2 */
ISR(PCINT2_vect)
{
    noInterrupts();

    // interrupt was triggered by OPENING the clapper
    // (clapper STATE currently CLOSED -- must be closed to open)
    if (!clapper_is_open) {
        Serial.println("OPEN CLAPPER");
        just_clapped = false;
        clapper_is_open = true;
    }
    // interrupt was triggered by CLOSING the clapper
    // (clapper STATE currently OPEN -- must be open to close)
    else {
        Serial.println("CLOSE CLAPPER");
        clapper_is_open = false;
        just_clapped = true;
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
    currentBit = 0;
    lastBit = 0;
    currentFrameIndex = 0;
    validFrameCount = 0;
    frameAvailable = false;
    state = NOSYNC;
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
    state = NOSYNC;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GENERATOR

// ISR(TIMER1_COMPA_vect)
// {
//     generator.interupt();
// }

// void startLTCGenerator()
// {
//     noInterrupts();
//     TCCR1A = 0; // clear all
//     TCCR1B = (1 << WGM12) | (1 << CS10);
//     TIMSK1 = (1 << OCIE1A);

//     TCNT1 = 0;

//     // 30 fps, 80 bits * 30
//     OCR1A = 3333; // = 16000000 / (30 * 80 * 2)

//     state = GENERATOR;
//     generator.reset();
//     interrupts();
// }

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// HELPERS

void flash_sync_indicator()
{
    LED.setDisplayArea(4);
    LED.displayOn();
    LED.print(".");
    delay(1000 / FRAME_RATE);
    LED.displayOff();
    LED.setDisplayArea(1, 2, 3, 4, 5, 6, 7, 8);
}

/* update the timecode string before printing
                    _ _ . _ _ . _ _ . _ _ \0
  char[12] index:   0 1 2 3 4 5 6 7 8 9 0 1
*/
void update_TC_string()
{
    sprintf(
        TC_string,
        "%02d.%02d.%02d.%02d",
        decoded.h, decoded.m, decoded.s, decoded.f);
}

/* update the user bits string before printing */
void update_UB_string()
{
    sprintf(
        UB_string,
        "%d%d.%d%d.%d%d.%d%d",
        decoded.ub0, decoded.ub1, decoded.ub2, decoded.ub3, decoded.ub4, decoded.ub5, decoded.ub6, decoded.ub7);
}

/* from the latest frame of LTC, decode time values into decimal integers */
/*
    Decode Binary Coded Decimal (BCD) values from specific bits of an LTC frame.

*/
void decode_and_update_time_vals(volatile byte* curr_frame_bytes)
{
    // 10s place + 1s place
    decoded.h = (curr_frame_bytes[7] & 0x03) * 10 + (curr_frame_bytes[6] & 0x0F); // 0x03 -> smallest 2 bits (2+1=3)
    decoded.m = (curr_frame_bytes[5] & 0x07) * 10 + (curr_frame_bytes[4] & 0x0F); // 0x07 -> smallest 3 bits (4+2+1=7)
    decoded.s = (curr_frame_bytes[3] & 0x07) * 10 + (curr_frame_bytes[2] & 0x0F); // 0x0F -> smallest 4 bits (8+4+2+1=F)
    decoded.f = (curr_frame_bytes[1] & 0x03) * 10 + (curr_frame_bytes[0] & 0x0F);

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
                - small 1/2 of 7th and 6th bytes of 8 byte (80 bit) long frame

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
void decode_UB_and_update_vals()
{
    /* rshift 4 bc, despite picking out the large half of LTC byte, we're
    using 1,2,4,8 places only, as that's all that's needed to make a hex char
    - (right shift is always towards LSB)
    - each ubX is 1 digit bt 0 and F (aka a hex char)
    - ...& 0xF0 -> access largest 4 bits of this LTC byte by masking off the
        smallest 4 bits */
    decoded.ub7 = (current_frame_bytes[0] & 0xF0) >> 4;
    decoded.ub6 = (current_frame_bytes[1] & 0xF0) >> 4;
    decoded.ub5 = (current_frame_bytes[2] & 0xF0) >> 4;
    decoded.ub4 = (current_frame_bytes[3] & 0xF0) >> 4;
    decoded.ub3 = (current_frame_bytes[4] & 0xF0) >> 4;
    decoded.ub2 = (current_frame_bytes[5] & 0xF0) >> 4;
    decoded.ub1 = (current_frame_bytes[6] & 0xF0) >> 4;
    decoded.ub0 = (current_frame_bytes[7] & 0xF0) >> 4;
}

void wait_for_display()
{
    // wait for led display to init
    while (LED.begin(LED.e8Bit) != 0) {
        Serial.println("Failed to initialize the chip, please confirm the chip connection!");
        delay(1000);
    }
}

/* print the SMPTE string (TC or UB) to the 8 digit, 7 segment, LED display */
void print_to_segment_display(char* SMPTE_string)
{
    LED.print(
        &SMPTE_string[0], &SMPTE_string[1], // hours, ones place + '.'
        &SMPTE_string[3], &SMPTE_string[4], // minutes, "
        &SMPTE_string[6], &SMPTE_string[7], // seconds, "
        &SMPTE_string[9], &SMPTE_string[10] // frames, "
    );
}

void freeze_display()
{
    // copy the current TC and UB to strings that will be held on the display
    strcpy(TC_on_clap, TC_string);
    strcpy(UB_on_clap, UB_string);

    print_to_segment_display(TC_on_clap);
    delay(DISPLAY_HOLD_MILLISEC);

    print_to_segment_display(UB_on_clap);
    delay(DISPLAY_HOLD_MILLISEC);
}

void handle_clap()
{
    // LED.displayOff();
    just_clapped = false; // reset for next clap
    freeze_display();
}

void handle_open_clapper()
{
    LED.displayOn();
    print_to_segment_display(TC_string);
}

/* listen on pin D2 for state change (of hall sensor) to trigger an interrupt
(IOW, set up a [P]in [C]hange [I]nterrupt)
    PCICR = pin change interrupt control register
    PCMSK2 = pin change interrupt mask, group 2) */
void setup_hall_sensor_for_pin_change_interrupt()
{
    // listen for state change on interrupt group 2 (port D) pins
    PCICR |= B00000100;
    // listen only on pin D2 (aka pin2 grp2) for state changes
    PCMSK2 |= B00000100;
    pinMode(HALL_SENSOR_PIN, INPUT);
}

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

/*
    HOW ARE CHARACTERS PRINTED ON THE SEGMENTED DISPLAY?
    A  ->  0x77  ->  0111 0111
    B  ->  0x7C  ->  0111 1100
    C  ->  0x39  ->  0011 1001
    D  ->  0x5E  ->  0101 1110
    E  ->  0x79  ->  0111 1001
    F  ->  0x71  ->  0111 0001
    .  ->  0x80  ->  1000 0000

            LED Segment Display                 LETTER A (LSB)
                     A                             bit 0
                    ----                           ####
               F  -      -  B            bit 5   #      #  bit 1
                  -      -                       #      #
             G -->  ----                           ####   <-- bit 6
                  -      -               bit 4   #      #  bit 2
               E  -      -  C                    #      #
                    ----       #                   ----       -
                      D        DP                  bit 3      bit 7
 */

/* TODO:
- debounce hall sensor
- redo the state machine
- implement:
    // flash the given value on the display
    void flash_display(char *str_8_dig)

    // determines the framerate and informs the user via the segmented display
    void report_framerate()
- bugfixes:
    - display never re-initializes if its input stream is interrupted
        - how would a display input interruption be detected?
*/