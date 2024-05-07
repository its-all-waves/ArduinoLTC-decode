#include "LTCFrame.h"

enum ReaderState {
    NO_SYNC,
    SYNC
};

struct TC {
    // hours, minutes, seconds, frames
    uint8_t h = 0, m = 0, s = 0, f = 0;
};

struct UB {
    uint8_t ub0 = 0, ub1 = 0, ub2 = 0, ub3 = 0,
            ub4 = 0, ub5 = 0, ub6 = 0, ub7 = 0;
};

// If the time since the LTC signal's phase change is < this, it's not a valid signal
#define BIT_TIME_MIN 250
// If the time since the LTC signal's phase change is > this, it's not a valid signal
#define BIT_TIME_MAX 1500
/* If the time since the LTC signal's phase change is > this, the current bit
must be a 1. If longer, it must be a 0. */
#define BIT_TIME_THRESHOLD 700

#define LAST_LTC_BIT_INDEX 79

class LTCReader {
public: // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    volatile boolean is_new_frame = false; // true when received last bit of a frame
    unsigned long running_frame_count = 0; // resets when sync is lost

    ReaderState get_state()
    {
        return state;
    }

    char* get_tc_string()
    {
        sprintf(
            tc_string,
            "%02d.%02d.%02d.%02d",
            tc.h, tc.m, tc.s, tc.f);
        return tc_string;
    }

    /* Update the user bits string before printing */
    char* get_ub_string()
    {
        sprintf(
            ub_string,
            "%d%d.%d%d.%d%d.%d%d",
            ub.ub0, ub.ub1, ub.ub2, ub.ub3,
            ub.ub4, ub.ub5, ub.ub6, ub.ub7);
        return ub_string;
    }

    boolean is_new_second()
    {
        return tc.f == 0;
    }

    /* Returns true if at least one valid frame has been read. */
    boolean is_reading_valid_frames()
    {
        return running_bit_count > 80;
    }

    void reset()
    {
        running_bit_count = 0;
        time_since_last_phase = 0;
        sync_pattern_comparator = 0;
        currentFrameIndex = 0;
        running_frame_count = 0;
        is_new_frame = false;
        state = NO_SYNC;
    }

    /*
    Decode Binary Coded Decimal (BCD) values from specific bits of an LTC frame
    and store the values in the decodedLTC struct.
    */
    void decode_tc()
    {
        byte* curr_frame = frame_buf[1 - currentFrameIndex];

        // 10s place + 1s place
        tc.h = (curr_frame[7] & 0x03) * 10 + (curr_frame[6] & 0x0F); // 0x03 -> smallest 2 bits (2+1=3)
        tc.m = (curr_frame[5] & 0x07) * 10 + (curr_frame[4] & 0x0F); // 0x07 -> smallest 3 bits (4+2+1=7)
        tc.s = (curr_frame[3] & 0x07) * 10 + (curr_frame[2] & 0x0F); // 0x0F -> smallest 4 bits (8+4+2+1=F)
        tc.f = (curr_frame[1] & 0x03) * 10 + (curr_frame[0] & 0x0F);

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
        byte* curr_frame = frame_buf[1 - currentFrameIndex];

        ub.ub7 = (curr_frame[0] & 0xF0) >> 4;
        ub.ub6 = (curr_frame[1] & 0xF0) >> 4;
        ub.ub5 = (curr_frame[2] & 0xF0) >> 4;
        ub.ub4 = (curr_frame[3] & 0xF0) >> 4;
        ub.ub3 = (curr_frame[4] & 0xF0) >> 4;
        ub.ub2 = (curr_frame[5] & 0xF0) >> 4;
        ub.ub1 = (curr_frame[6] & 0xF0) >> 4;
        ub.ub0 = (curr_frame[7] & 0xF0) >> 4;

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

    /* Currently a misnomer. Returns true if state changes or reached the end of
    a valid frame. */
    boolean state_changed()
    {
        switch (state) {
        case NO_SYNC:
            if (sync_pattern_comparator == SYNC_PATTERN) {
                // sync pattern detected
                state = SYNC;
                bits_read_of_curr_frame = 0;
                return true;
            }
            break;
        case SYNC:
            if (bits_read_of_curr_frame > LAST_LTC_BIT_INDEX
                && sync_pattern_comparator != SYNC_PATTERN) {
                // reached the end of a frame but didn't see the sync pattern,
                reset();
                return true;
            }

            // if this is the last bit of a frame
            if (sync_pattern_comparator == SYNC_PATTERN) {
                is_new_frame = true; // signal that we've captured a full frame
                running_frame_count++;
                bits_read_of_curr_frame = 0; // reset

                // alternates between 0 and 1: if 1, becomes 0; if 0, becomes 1
                currentFrameIndex = 1 - currentFrameIndex;

                return true;
            }
            break;
        }
        return false;
    }

    /* Store the new LTC bit in the current frame. The main work of LTCReader.
    @param counter_ticks_since_LTC_signal_phase_change The "time" of the phase change of the LTC
    signal, or the time of the interrupt that triggered the update. The time is
    represented by a hardware timer/counter value. */
    void update(unsigned int counter_ticks_since_LTC_signal_phase_change)
    {
        time_since_last_phase = counter_ticks_since_LTC_signal_phase_change;

        // reset on signal loss
        if (time_since_last_phase < BIT_TIME_MIN || time_since_last_phase > BIT_TIME_MAX) {
            reset();
            return;
        }

        running_bit_count++;

        /*
        In the LTC modulation scheme, "1" bit has 2 transitions per bit period where
        a "0" bit has 1 transition. If the measured period between transitions is
        greater than that required to make a 1 bit, it must be a 0 bit.
        */
        current_bit_val = time_since_last_phase > BIT_TIME_THRESHOLD
            ? 0
            : 1;

        // don't count 1 twice! TODO: What does this do? DUNNO but it breaks without it!
        if (current_bit_val == 1 && last_bit_val == 1) {
            last_bit_val = 0;
            return;
        }

        last_bit_val = current_bit_val;

        // update frame sync pattern detection
        sync_pattern_comparator = (sync_pattern_comparator >> 1) + (current_bit_val << 15);

        if (state_changed())
            return;

        // we're on a bit between the start & end of the current frame

        byte* f = frame_buf[currentFrameIndex]; // grab all the bits we've recorded for this frame so far

        // TODO: why is this different from currentFrameIndex?
        // used to update ltc data in the ICP1 interrupt (LTC input) (decode mode)
        byte idx = bits_read_of_curr_frame / 8; // get the index of a byte from frames[]
        byte bIdx = bits_read_of_curr_frame & 0x07; // the bit index of this frame

        // update the current bit (in its byte at f[idx]) with the value of current_bit_val
        f[idx] = (f[idx] & ~(1 << bIdx)) | (current_bit_val << bIdx);
        bits_read_of_curr_frame++;

        return;
    }

private: // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ReaderState state = NO_SYNC;

    TC tc;
    UB ub;

    char* tc_string = "00.00.00.00";
    char* ub_string = "00.00.00.00";

    /* The LTC spec's sync word: fixed bit pattern 0011 1111 1111 1101.
    Used to detect the end of a frame. */
    const uint16_t SYNC_PATTERN = 0xBFFC;
    /* Read from incoming LTC. When matches SYNC_PATTERN, indicates end of a
    frame (is_new_frame = true) */
    volatile uint16_t sync_pattern_comparator = 0x0;

    // counts bits up to 80, resets for each frame read
    volatile uint8_t bits_read_of_curr_frame = 0;
    volatile uint16_t running_bit_count = 0;

    // used to determine if current bit is 1 or 0
    volatile unsigned int time_since_last_phase = 0;

    /*
    Store 2 frames -- TODO: WHY 2? why not initialize to all zeros, except sync pattern?
    NOTE: each frame is 10 bytes or 80 bits (the LTC standard). The last two
    bytes of each frame is the sync pattern.
    */
    LTCFrame frame_buf[2] = {
        { 0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF },
        { 0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF }
    };
    byte currentFrameIndex = 0; // index of frames[2] -- can be 0 or 1 (ASSUMPTION)

    volatile boolean current_bit_val, last_bit_val;
};