#include "DFRobot_LedDisplayModule.h"

#define ICP1 8 // ICP1, 8 for atmega368, 4 for atmega32u4
#define LTC_OUT 9
#define SIGNAL_LED 13
#define LOCK_LED 6

/* TODO: how are these BIT_TIME_X constants used?
    seems to be an error check
        if the width in microsec of a decoded bit is too short or long to be
            considered an LTC bit 
    if this was the case, wouldn't we have to measure the time between
        hi/lo transitions somewhere?
            is that happening somewhere?
 */
#define BIT_TIME_THRESHOLD 700
#define BIT_TIME_MIN 250
#define BIT_TIME_MAX 1500

// states of the machine
#define NOSYNC 0
#define SYNCED 1
#define GENERATOR 2

// prototypes
void decode_and_update_time_vals(),
    update(char* TC_string),
    decode_UB_and_update(char* UB_string),
    print_to_segment_display(char* TC_string);

void startLTCDecoder(),
    stopLTCDecoder(),
    startLTCGenerator();


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// GLOBAL VARIABLES

// instantiate segmented display object with the IIC address of the display
DFRobot_LedDisplayModule LED(&Wire, 0xE0);

// the timecode string to print: 8 digits + 3 separators (.) + \0
char TC_string[12] = {
    '0', '0', '.', '0', '0', '.', '0', '0', '.', '0', '0', '\0'
};

// the user bits string to print: same format as LTC_string
char UB_string[12] = {
    '0', '0', '.', '0', '0', '.', '0', '0', '.', '0', '0', '\0'
};

// the time values (in decimal) extracted from LTC in the decoder
uint8_t h,
    m,
    s,
    f;

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

            bitIndex ++;
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

            frame ++;
            if (frame > 30) {
                seconds ++;
                frame = 0;
            }

            if (seconds > 60) {
                minutes ++;
                seconds = 0;
            }

            if (minutes > 60) {
                hours ++;
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


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// MAIN ARDUINO FUNCTIONS

void setup()
{
    Serial.begin(115200);
    pinMode(ICP1, INPUT); // ICP pin (digital pin 8 on arduino) as input
    pinMode(LTC_OUT, OUTPUT);

    pinMode(SIGNAL_LED, OUTPUT);
    pinMode(LOCK_LED, OUTPUT);

    digitalWrite(SIGNAL_LED, LOW);
    digitalWrite(LOCK_LED, LOW);

    while (!Serial) {
        // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("Ready!");

    // wait for the LED display connection to initialize
    while (LED.begin(LED.e8Bit) != 0) {
        Serial.println("Failed to initialize the chip, please confirm the chip connection!");
        delay(1000);
    }

    // make all 8 digits available on the display
    LED.setDisplayArea(1, 2, 3, 4, 5, 6, 7, 8);

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

    // mode of operation is DECODER
    // state is NOSYNC or SYNCED

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
    update(TC_string);

    // update the user bits string (but don't display it until appropriate)
    decode_UB_and_update(UB_string);

    // print to segmented LED display + serial monitor
    // print_to_segment_display(TC_string);
    print_to_segment_display(UB_string);
    // Serial.println(TC_string);
    Serial.println(UB_string);

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
            validFrameCount ++;

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
        if(currentBit)
            f[idx] |= 1 << bIdx;
            else
            f[idx] &= ~(1 << bIdx);
        */

        frameBitCount ++;
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

/* recreate the timecode string to print
                    _ _ . _ _ . _ _ . _ _ \0
  char[12] index:   0 1 2 3 4 5 6 7 8 9 0 1
*/
void update(char* TC_string)
{
    sprintf(TC_string, "%02d.%02d.%02d.%02d", h, m, s, f);
}

/* decode time vals from the latest frame's LTC into decimal integers */
void decode_and_update_time_vals() {
    // 10s place + 1s place
    h = (fptr[7] & 0x03) * 10 + (fptr[6] & 0x0F); // 0x03 -> smallest 2 bits (2+1=3) 
    m = (fptr[5] & 0x07) * 10 + (fptr[4] & 0x0F); // 0x07 -> smallest 3 bits (4+2+1=7)
    s = (fptr[3] & 0x07) * 10 + (fptr[2] & 0x0F); // 0x0F -> smallest 4 bits (8+4+2+1=F)
    f = (fptr[1] & 0x03) * 10 + (fptr[0] & 0x0F);
}

/* decode userbit hex vals from the bytes of an LTC frame and use them to update
the user bits string to be displayed */
void decode_UB_and_update(char* UB_string) {

        // lookup table for decoding user bits from binary
        #define HEX_CHARS "0123456789ABCDEF"

        /* rshift 4 bc, despite picking out the large half of LTC byte, we're
        using 1,2,4,8 places only, as that's all that's needed to make a hex
        (right shift is always towards LSB) 
        & 0xF0 -> access largest 4 bits of this LTC byte by masking off the
        smallest 4 bits */
        uint8_t ub7 = (fptr[0] & 0xF0) >> 4; // 1 digit bt 0 and F
        uint8_t ub6 = (fptr[1] & 0xF0) >> 4; // 1 digit bt 0 and F
        uint8_t ub5 = (fptr[2] & 0xF0) >> 4; // 1 digit bt 0 and F
        uint8_t ub4 = (fptr[3] & 0xF0) >> 4; // 1 digit bt 0 and F
        uint8_t ub3 = (fptr[4] & 0xF0) >> 4; // 1 digit bt 0 and F
        uint8_t ub2 = (fptr[5] & 0xF0) >> 4; // 1 digit bt 0 and F
        uint8_t ub1 = (fptr[6] & 0xF0) >> 4; // 1 digit bt 0 and F
        uint8_t ub0 = (fptr[7] & 0xF0) >> 4; // 1 digit bt 0 and F
    
        // set this UB character to the hex char from this UB field
        UB_string[0]  =  HEX_CHARS[ub0];
        UB_string[1]  =  HEX_CHARS[ub1];
        UB_string[3]  =  HEX_CHARS[ub2];
        UB_string[4]  =  HEX_CHARS[ub3];
        UB_string[6]  =  HEX_CHARS[ub4];
        UB_string[7]  =  HEX_CHARS[ub5];
        UB_string[9]  =  HEX_CHARS[ub6];
        UB_string[10] =  HEX_CHARS[ub7];



    // from the latest frame, decode user bit binary into hex values
    // user bit field/char (starting at 0) aligns with LTC byte #
    // #define TOTAL_DISPLAYED_CHARS 7 // TODO: is this 8 or 12? 8 chars + 3 separators (.)
    // for (uint8_t i = 0; i < TOTAL_DISPLAYED_CHARS; i ++) {
    //     // TODO: adjust the UB_string indices to account for separators
    //     // continue if index is a separator? (quick thought, maybe crap)



    //     if (i == 2 || i == 5 || i == 8 || i == 11)
    //         continue;

    //     // TODO: MSB or LSB?
    //     /* rshift 4 bc, despite picking out the large half of LTC byte, we're
    //     using 1,2,4,8 places only, as that's all that's needed to make a hex
    //     (right shift is always towards LSB) 
    //     & 0xF0 -> access largest 4 bits of this LTC byte by masking off the
    //     smallest 4 bits */
    //     uint8_t decoded_hex_val = (fptr[i] & 0xF0) >> 4; // 1 digit bt 0 and F
        
    //     // // DEBUG
    //     Serial.print("char @ i = ");
    //     Serial.print(decoded_hex_val);
    //     Serial.println();

    //     // set this UB character to the hex char from this UB field
    //     UB_string[i] = HEX_CHARS[decoded_hex_val];
        
    //     // // // DEBUG
    //     // Serial.print("char @ i = ");
    //     // Serial.print(UB_string[i]);
    //     // Serial.println();

    // }
}

/* print the LTC string to the 8 digit, 7 segment, LED display */
void print_to_segment_display(char* TC_string)
{
    LED.print(
        &TC_string[0], &TC_string[1], // hours, ones place + '.'
        &TC_string[3], &TC_string[4], // minutes, "
        &TC_string[6], &TC_string[7], // seconds, "
        &TC_string[9], &TC_string[10] // frames, "
    );
}


/* TODO: USER BITS

    Thirty two bits are assigned as eight groups of four USER BITS
    each of the 8 UB characters can be a single digit
        or a letter (A - F (65-70 ASCII) on Sound Devices 664)

    32 bits = 4 bytes
    1 byte = max value 255

    * * * TODO: MSB or LSB? * * * 
    spec says LSB, but everything else is MSB... (confirm the latter)

    ASSUMPTION: since a full byte can count to 15, and we only need 0-9 for
    single digits, the values 10-15 must be...
        10=A, 11=B, 12=C, 13=D, 14=E, 15=F

    e.g. how do I say the date and reel?
        2023 Aug 7
        sound reel #3F
        
    GOAL USER BITS (UB)   23 : 08 : 07 : 3F -->

          2        3 :      0        8 :      0        7 :      3        F  <- UB goal
    0 0 1 0  0 0 1 1  0 0 0 0  1 0 0 0  0 0 0 0  0 1 1 1  0 0 1 1  0 1 1 0  <- LTC stream
    - - - -  - - - -  - - - -  - - - -  - - - -  - - - -  - - - -  - - - -
    8 4 2 1  8 4 2 1  8 4 2 1  8 4 2 1  8 4 2 1  8 4 2 1  8 4 2 1  8 4 2 1  <- place - what should these values be?
                   1                 2                 3                 4  <- UB byte number          
          1        2        3        4        5        6        7        8  <- UB character


    FIGURE IT OUT

    1) where are the user bit fields?

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

    2) on which end of the UB string is field/char 1?

        ? ? ?

    3) how do we access the bits in the 2nd half of the data bytes 1-8
       (9-10 is the sync word)


    4) how do we 



    THE INTENTION OF THE USERBITS DECODING SECTION - IS IT CORRECT?

    looks at the first 8 bytes of a 10-byte schema and extracts the last 4
    bits of each byte. Specifically, it is using a right shift operation (>> 4)
    to move the last 4 bits of each byte into the first 4 positions, and then
    using a bitwise AND operation (& 0x0F) to mask out all but the last 4 bits.
    The resulting value is then used as an index into the string
    "0123456789ABCDEF" to convert the binary value into a hexadecimal character.
    This process is repeated for each of the first 8 bytes in the schema, and
    the resulting hexadecimal characters are stored in the userBitsString array.
*/