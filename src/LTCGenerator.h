
#include "Arduino.h"
#include "LTCFrame.h"

struct LTCGenerator {
    volatile byte bitIndex;
    volatile byte bitToggle;
    volatile LTCFrame outFrames[2];
    volatile byte currentFrameIndex; // current frame read by ISR
    volatile boolean generateNewFrame;

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

                // switch frame
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