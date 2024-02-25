#include "DFRobot_LedDisplayModule.h"
#include <string.h>

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
void decode_and_update_time_vals(),
    decode_UB_and_update_vals(),
    update_TC_string(),

    // segmented display
    wait_for_display(),
    print_to_segment_display(char* SMPTE_string),
    flash_sync_indicator(),

    // main modes of the machine
    startLTCDecoder(),
    stopLTCDecoder(),
    startLTCGenerator(),

    // hall effect sensor
    setup_hall_sensor_for_pin_change_interrupt(),
    freeze_display(),
    handle_clap();

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GLOBAL VARIABLES

// instantiate segmented display object with the IIC address of the display
DFRobot_LedDisplayModule LED(&Wire, 0xE0);

// the timecode string to print: 8 digits + 3 separators (.) + \0
// this format (with periods) is required by the DFRobot segment display lib
char TC_string[12] = {
    '0', '0', '.', '0', '0', '.', '0', '0', '.', '0', '0', '\0'
};

// the user bits string to print: same format as SMPTE_string
char UB_string[12] = {
    '0', '0', '.', '0', '0', '.', '0', '0', '.', '0', '0', '\0'
};

char sync_indicator_str[12] = {
    82, 82, 82, 82, 82, '.', 82, 82, 82, 82, 82, '\0'
};

// char* sync_indicator_str = "     .     ";

// char sync_indicator_str[8] = { 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80
// };

// char sync_indicator_str[2] = { '.', '\0' };

// the time values (in decimal) extracted from LTC in the decoder
uint8_t h,
    m, s, f;

// user bit fields/chars extracted from LTC in the decoder
uint8_t ub7, ub6, ub5, ub4, ub3, ub2, ub1, ub0;

// 10 bytes * 8 bits = 80 bits = SMPTE/LTC frame length
typedef byte LTCFrame[10];

// store 2 frames -- TODO: WHY 2? why not initialize to all zeros?
volatile LTCFrame frames[2] = {
    { 0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF },
    { 0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF }
};
volatile byte currentFrameIndex; // index of frames[2] -- can be 0 or 1 (ASSUMPTION)

volatile boolean frameAvailable; // indicates received last bit of an LTC frame

// the LTC spec's sync word: fixed bit pattern 0011 1111 1111 1101
const unsigned short syncPattern = 0xBFFC;
// read from incoming LTC
// when matches syncPattern, indicates end of a frame (frameAvailable = true)
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

// used to update ltc data in the ISR for capture (decode mode)
byte idx, bIdx;

byte* fptr; // index into this to access the bytes of the current LTC frame (decode mode)

int previousOutputFrameIndex = 0;

struct LTCGenerator {
    volatile byte bitIndex;
    volatile byte bitToggle;
    volatile LTCFrame outFrames[2];
    volatile byte currentFrameIndex; // current frame read by ISR
    volatile bool generateNewFrame;

    int frame;
    int seconds;
    int minutes;
    int hours;

    LTCGenerator()
    {
        reset();
    }

    void reset()
    {
        bitIndex = 0;
        bitToggle = 0;
        currentFrameIndex = 0;

        // init frame data
        memset(outFrames[0], 0, 10);
        outFrames[0][8] = 0xFC;
        outFrames[0][9] = 0xBF; // sync pattern

        memset(outFrames[1], 0, 10);
        outFrames[1][8] = 0xFC;
        outFrames[1][9] = 0xBF; // sync pattern

        generateNewFrame = false;

        frame = 0;
        seconds = 0;
        minutes = 0;
        hours = 0;
    }

    // send current frame (should be call inside an ISR)
    void interupt()
    {
        // toggle for 0 and 1;
        if (!bitToggle) {
            PORTB ^= (1 << 5);

            if (bitIndex == 0)
                PORTD ^= (1 << 7); // debug, use to trigger the scope
        } else {
            byte idx = bitIndex / 8;
            byte bitIdx = bitIndex & 0x07;
            byte value = outFrames[currentFrameIndex][idx] & (1 << bitIdx);

            // toggle for 1
            if (value)
                PORTB ^= (1 << 5);

            bitIndex++;
            if (bitIndex >= 80) {
                // reset read position
                bitIndex = 0;

                // swtich frame
                currentFrameIndex = 1 - currentFrameIndex;

                generateNewFrame = true;
            }
        }

        bitToggle ^= 1; // toggle
    }

    void update()
    {
        if (generateNewFrame) {
            generateNewFrame = false;

            frame++;
            if (frame > 30) {
                seconds++;
                frame = 0;
            }

            if (seconds > 60) {
                minutes++;
                seconds = 0;
            }

            if (minutes > 60) {
                hours++;
                minutes = 0;
            }

            // generate frame data
            outFrames[1 - currentFrameIndex][0] = (frame % 10) & 0xf;
            outFrames[1 - currentFrameIndex][1] = (frame / 10) & 0x3;
            outFrames[1 - currentFrameIndex][2] = (seconds % 10) & 0xf;
            outFrames[1 - currentFrameIndex][3] = (seconds / 10) & 0x7;
            outFrames[1 - currentFrameIndex][4] = (minutes % 10) & 0xf;
            outFrames[1 - currentFrameIndex][5] = (minutes / 10) & 0x7;
            outFrames[1 - currentFrameIndex][6] = (hours % 10) & 0xf;
            outFrames[1 - currentFrameIndex][7] = (hours / 10) & 0x3;
        }
    }

} generator;

// hall effect sensor macros and globals
#define HALL_SENSOR_PIN 2
#define DISPLAY_HOLD_MILLISEC 3000
volatile bool just_clapped = false, // true -> display TC_on_clap
    clapper_is_open; // true -> display on
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

    pinMode(ICP1, INPUT); // ICP pin (digital pin 8 on arduino) as input
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

    if (state == GENERATOR) {
        generator.update();
        return;
    }

    // mode of operation is DECODER, state is NOSYNC or SYNCED

    // clapper handling (via hall effect sensor)
    if (just_clapped)
        handle_clap();

    if (clapper_is_open)
        handle_open_clapper();

    // only proceed to update time vars if we've past the end of an LTC frame
    if (!frameAvailable)
        return;

    // reached end of a frame, so there's new data to print
    fptr = frames[1 - currentFrameIndex]; // apparently this error can be ignored (?): a value of type "volatile byte *" cannot be assigned to an entity of type "byte *"C/C++(513)

    Serial.print("Frame: ");
    Serial.print(validFrameCount - 1);
    Serial.print(" - ");

    // decode time values and update the timecode string to be displayed
    decode_and_update_time_vals();
    update_TC_string();

    // update the user bits string (but don't display it until appropriate)
    decode_UB_and_update_vals();
    update_UB_string();

    // TODO: show "sync valid" indicator when on first frame (flash the 4th dot
    // on the display)
    if (f == 0 & !clapper_is_open) {
        Serial.println("SECOND MARK");
        // TODO: the following does not work!
        LED.setDisplayArea(4);
        LED.displayOn();
        LED.print(".");
        LED.displayOff();
        LED.setDisplayArea(1, 2, 3, 4, 5, 6, 7, 8);
    } else {
        // clapper is open and this is not the first frame of a second
        // print timecode to segmented LED display + serial monitor
        print_to_segment_display(TC_string);
        Serial.println(TC_string);
    }

    // reset
    frameAvailable = false;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// DECODER

/* triggered when a transition (?) is detected at the input capture pin */
ISR(TIMER1_CAPT_vect)
{
    TCCR1B ^= _BV(ICES1); // toggle edge capture
    bitTime = ICR1; // store counter value at edge
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
        if (syncValue == syncPattern) {
            state = SYNCED;
            frameBitCount = 0;
            return;
        }
        break;
    case SYNCED:
        if ((frameBitCount > 79 && syncValue != syncPattern)
            || (syncValue == syncPattern && frameBitCount != 79)) {
            // something went wrong!
            syncValue = 0;
            validBitCount = 0;
            validFrameCount = 0;
            frameAvailable = false;
            state = NOSYNC;
            return;
        }

        // if this is the last bit of a frame
        if (syncValue == syncPattern) {
            frameAvailable = true; // signal that we've captured a full frame

            // reset bit counter and increment total frame count
            frameBitCount = 0;
            validFrameCount++;

            // ??? TODO: is this a boolean -- only 0 or 1? frames[] holds only 2 LTC frames
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
    // disable global interrupts so this interruption isn't interrupted
    cli();

    // interrupt was triggered by OPENING the clapper
    // (clapper STATE currently CLOSED -- must be closed to open)
    if (!clapper_is_open) {
        Serial.println("OPEN CLAPPER");
        clapper_is_open = digitalRead(HALL_SENSOR_PIN);
        // ensure clapper_is_open is true
        /* TODO: I had a feeling that I should not arbitrarily set
        clapper_is_open in this if statement. I felt this var should always
        track the sensor reading, and I should assert that it is the expected
        value. Not sure if this is needed, but it seems to not hurt. */
        if (clapper_is_open != true) {
            Serial.println("ERROR 1");
            LED.print("Err 1   ");
            return;
        }
    }
    // interrupt was triggered by CLOSING the clapper
    // (clapper STATE currently OPEN -- must be open to close)
    else {
        Serial.println("CLOSE CLAPPER");
        clapper_is_open = digitalRead(HALL_SENSOR_PIN);
        if (clapper_is_open != false) {
            Serial.println("ERROR 2");
            LED.print("Err 2   ");
            return;
        }
        just_clapped = true;
    }

    // re-enable global interrupts
    sei();
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

    state = GENERATOR;
    generator.reset();
    interrupts();
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// HELPERS

void flash_sync_indicator()
{
    // show a period on LED display for X ms
    print_to_segment_display(sync_indicator_str);
    delay(500);
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
        h, m, s, f);
}

/* update the user bits string before printing */
void update_UB_string()
{
    sprintf(
        UB_string,
        "%d%d.%d%d.%d%d.%d%d",
        ub0, ub1, ub2, ub3, ub4, ub5, ub6, ub7);
}

/* from the latest frame of LTC, decode time values into decimal integers */
void decode_and_update_time_vals()
{
    // 10s place + 1s place
    h = (fptr[7] & 0x03) * 10 + (fptr[6] & 0x0F); // 0x03 -> smallest 2 bits (2+1=3)
    m = (fptr[5] & 0x07) * 10 + (fptr[4] & 0x0F); // 0x07 -> smallest 3 bits (4+2+1=7)
    s = (fptr[3] & 0x07) * 10 + (fptr[2] & 0x0F); // 0x0F -> smallest 4 bits (8+4+2+1=F)
    f = (fptr[1] & 0x03) * 10 + (fptr[0] & 0x0F);
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
    ub7 = (fptr[0] & 0xF0) >> 4;
    ub6 = (fptr[1] & 0xF0) >> 4;
    ub5 = (fptr[2] & 0xF0) >> 4;
    ub4 = (fptr[3] & 0xF0) >> 4;
    ub3 = (fptr[4] & 0xF0) >> 4;
    ub2 = (fptr[5] & 0xF0) >> 4;
    ub1 = (fptr[6] & 0xF0) >> 4;
    ub0 = (fptr[7] & 0xF0) >> 4;
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
    just_clapped = false; // reset for next clap
    freeze_display();
    LED.displayOff();
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
                   1     byte 1          04-07
                   2     byte 2          12-15
                   3     byte 3          20-23
                   4     byte 4          28-31
                   5     byte 5          36-39
                   6     byte 6          44-47
                   7     byte 7          52-55
                   8     byte 8          60-63

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
    // blink the center-most period once per second,
    // blink faster if no sync
    void indicate_sync()

    // flash the given value on the display
    void flash_display(char *str_8_dig)

    // determines the framerate and informs the user via the segmented display
    void report_framerate()
- bugfixes:
    - display never re-initializes if its input stream is interrupted
        - how would a display input interruption be detected?
*/