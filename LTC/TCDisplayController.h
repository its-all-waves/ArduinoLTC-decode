#include "DFRobot_LedDisplayModule.h"

#define BOOTUP_STRING "--.--.--.--"

class DFRobot_8DigSegDisplController {
private:
    // instantiate segmented display object with the IIC address of the display
    DFRobot_LedDisplayModule display = DFRobot_LedDisplayModule(&Wire, 0xE0);

public:
    void init_display()
    {
        // wait for led display to init
        while (display.begin(display.e8Bit) != 0) {
            Serial.println("Failed to initialize the display, please confirm the display connection!");
            delay(1000);
        }
        display.setDisplayArea(1, 2, 3, 4, 5, 6, 7, 8);
        set_brightness(1);
        display_on();

        // indicate that the display has been initialized
        print(BOOTUP_STRING);
        delay(1000);
    }

    void display_on() { display.displayOn(); }

    void display_off() { display.displayOff(); }

    void print(char* LTC_string)
    {
        display.print(
            &LTC_string[0], &LTC_string[1], // hours, ones place + '.'
            &LTC_string[3], &LTC_string[4], // minutes, "
            &LTC_string[6], &LTC_string[7], // seconds, "
            &LTC_string[9], &LTC_string[10] // frames, "
        );
    }

    void set_brightness(uint8_t level)
    {
        // clamp level to between 1 and 8
        display.setBrightness(
            level < 1
                ? 1
                : level > 8
                ? 8
                : level);
    }

    void flash_sync_indicator(uint8_t frame_rate)
    {
        display.displayOn();
        display.print(" ", " ", " ", ".", " ", " ", " ", " ");
        delay(1000 / frame_rate);
        display.displayOff();
    }

    void freeze_display(char* tc_or_ub, int duration)
    {
        print(tc_or_ub);
        delay(duration);
    }
};

/*
    TODO: is this still relevant?

    HOW ARE CHARACTERS PRINTED ON A SEGMENTED DISPLAY?
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