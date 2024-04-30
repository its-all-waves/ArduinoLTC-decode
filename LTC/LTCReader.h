#include "LTCFrame.h"

typedef struct TC {
    uint8_t h = 0, m = 0, s = 0, f = 0;
};

typedef struct UB {
    uint8_t ub0 = 0, ub1 = 0, ub2 = 0, ub3 = 0,
            ub4 = 0, ub5 = 0, ub6 = 0, ub7 = 0;
};

/*
A misnomer - there is no clock to speak of. Refers to mutually
exclusive states related to reading and generating an LTC signal.
*/
typedef enum ReaderState {
    NO_SYNC,
    SYNC,
    // GENERATE // NO GENERATING YET!
};

#define BIT_TIME_THRESHOLD 700
#define BIT_TIME_MIN 250
#define BIT_TIME_MAX 1500
#define LAST_LTC_BIT_INDEX 79

class LTCReader {
public:
    ReaderState state = NO_SYNC;

    TC tc;
    UB ub;

    // the timecode string to print: 8 digits + 3 separators (.) + \0
    // this format (with periods) is required by the DFRobot segment display lib
    char* tc_string = "00.00.00.00";

    // the user bits string to print: same format as SMPTE_string
    char* ub_string = "UB.UB.UB.UB";

    volatile unsigned short validBitCount; //            " "            bits decoded

    volatile boolean frameAvailable; // indicates received last bit of an frame

    volatile byte* current_frame_bytes; // index into this to access the bytes of the current LTC frame (decode mode)

    /*
    Store 2 frames -- TODO: WHY 2? why not initialize to all zeros?
    NOTE: each frame is 10 bytes or 80 bits (the LTC standard). The last two
    bytes of each frame is the sync pattern.
    */
    LTCFrame frames[2] = {
        { 0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF },
        { 0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF }
    };
    byte currentFrameIndex; // index of frames[2] -- can be 0 or 1 (ASSUMPTION)

    unsigned long validFrameCount; // running counter of valid frames decoded

    volatile unsigned int bitTime; // TODO: is this the width of the LTC bit in time? as in, 1sec / frame_rate / LTC's_80bits

private:
    /* The LTC spec's sync word: fixed bit pattern 0011 1111 1111 1101.
    Used to detect the end of a frame. */
    const unsigned short SYNC_PATTERN = 0xBFFC;
    /* Read from incoming LTC. When matches SYNC_PATTERN, indicates end of a
    frame (frameAvailable = true) */
    volatile unsigned short syncValue;

    // counts bits up to 80, resets upon frameAvailable (got last bit of a frame)
    volatile byte frameBitCount;

    volatile byte oneFlag = 0;

    volatile boolean curr_bit_val, last_bit_val;

    /* Update the timecode string before printing */
    void update_TC_string()
    {
        sprintf(
            tc_string,
            "%02d.%02d.%02d.%02d",
            tc.h, tc.m, tc.s, tc.f);
    }

    /* Update the user bits string before printing */
    void update_UB_string()
    {
        sprintf(
            ub_string,
            "%d%d.%d%d.%d%d.%d%d",
            ub.ub0, ub.ub1, ub.ub2, ub.ub3,
            ub.ub4, ub.ub5, ub.ub6, ub.ub7);
    }

public:
    LTCReader() { }

    void reset()
    {
        validBitCount = 0;
        bitTime = 0;
        syncValue = 0;
        currentFrameIndex = 0;
        validFrameCount = 0;
        frameAvailable = false;
        state = NO_SYNC;
    }

    void read_LTC_signal(unsigned int ICR_val)
    {
        bitTime = ICR_val; // store counter value at edge / ICR = input capture register // TODO: verify comment is correct

        // remove out of range value // TODO: make this comment better
        if (bitTime < BIT_TIME_MIN || bitTime > BIT_TIME_MAX) {
            reset();
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
            return; // TODO: return what? does the caller return when this line returns?
        }

        last_bit_val = curr_bit_val;

        // update frame sync pattern detection
        syncValue = (syncValue >> 1) + (curr_bit_val << 15);

        if (state_changed())
            return;

        // we're on a bit between the start & end of the current frame

        // ptr to the current frame in the array of LTC frames (doesn't compile if global)
        byte* f = frames[currentFrameIndex]; // (grab all the bits we've gathered for this frame so far)

        // TODO: why is this different from currentFrameIndex?
        // used to update ltc data in the ICP1 interrupt (LTC input) (decode mode)
        byte idx = frameBitCount / 8; // get the index of a byte from frames[]
        byte bIdx = frameBitCount & 0x07; // the bit index of this frame

        // update the current bit (in its byte at f[idx]) with the value of curr_bit_val
        f[idx] = (f[idx] & ~(1 << bIdx)) | (curr_bit_val << bIdx);

        frameBitCount++;
        return;
    }

    /*
    Decode Binary Coded Decimal (BCD) values from specific bits of an LTC frame
    and store the values in the decodedLTC struct.
    */
    void decode_tc()
    {
        // 10s place + 1s place
        tc.h = (current_frame_bytes[7] & 0x03) * 10 + (current_frame_bytes[6] & 0x0F); // 0x03 -> smallest 2 bits (2+1=3)
        tc.m = (current_frame_bytes[5] & 0x07) * 10 + (current_frame_bytes[4] & 0x0F); // 0x07 -> smallest 3 bits (4+2+1=7)
        tc.s = (current_frame_bytes[3] & 0x07) * 10 + (current_frame_bytes[2] & 0x0F); // 0x0F -> smallest 4 bits (8+4+2+1=F)
        tc.f = (current_frame_bytes[1] & 0x03) * 10 + (current_frame_bytes[0] & 0x0F);

        update_TC_string();

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
    void decode_ub()
    {
        /* rshift 4 bc, despite picking out the large half of LTC byte, we're
        using 1,2,4,8 places only, as that's all that's needed to make a hex char
        - (right shift is always towards LSB)
        - each ubX is 1 digit bt 0 and F (aka a hex char)
        - ...& 0xF0 -> access largest 4 bits of this LTC byte by masking off the
            smallest 4 bits */
        ub.ub7 = (current_frame_bytes[0] & 0xF0) >> 4;
        ub.ub6 = (current_frame_bytes[1] & 0xF0) >> 4;
        ub.ub5 = (current_frame_bytes[2] & 0xF0) >> 4;
        ub.ub4 = (current_frame_bytes[3] & 0xF0) >> 4;
        ub.ub3 = (current_frame_bytes[4] & 0xF0) >> 4;
        ub.ub2 = (current_frame_bytes[5] & 0xF0) >> 4;
        ub.ub1 = (current_frame_bytes[6] & 0xF0) >> 4;
        ub.ub0 = (current_frame_bytes[7] & 0xF0) >> 4;

        update_UB_string();

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
    };

    /* Currently a misnomer. Returns true if state changes or reached the end of
    a valid frame. */
    boolean state_changed()
    {
        switch (state) {
        case NO_SYNC:
            // sync pattern detected
            if (syncValue == SYNC_PATTERN) {
                state = SYNC;
                frameBitCount = 0;
                return true;
            }
            break;
        case SYNC:
            // if it seems we reached the end of a frame but didn't see the sync pattern,
            // or if we see a sync pattern but have not counted
            if ((frameBitCount > LAST_LTC_BIT_INDEX && syncValue != SYNC_PATTERN)
                /* || (syncValue == SYNC_PATTERN && frameBitCount != LAST_LTC_BIT_INDEX) */) {
                // something went wrong!
                reset();
                return true;
            }

            // if this is the last bit of a frame
            if (syncValue == SYNC_PATTERN) {
                frameAvailable = true; // signal that we've captured a full frame
                frameBitCount = 0; // reset
                validFrameCount++;

                // alternates between 0 and 1: if 1, becomes 0; if 0, becomes 1
                currentFrameIndex = 1 - currentFrameIndex;

                return true;
            }
            break;
        }
        return false;
    }
};