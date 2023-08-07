#include "DFRobot_LedDisplayModule.h"

#define FRAME_RATE 24

#define increment_frames() f++
void count_timecode(uint8_t& h, uint8_t& m, uint8_t& s, uint8_t& f),
    update(char* LTC_string),
    print_to_display(char* t);

// time units
uint8_t h = 0,
        m = 0,
        s = 0,
        f = 0;

// instantiate an LED object with the IIC address of the display
DFRobot_LedDisplayModule LED(&Wire, 0xE0);

// the timecode to print (string): 8 digits + 3 separators (. or :) + \0
char LTC_string[12];

void setup()
{
    Serial.begin(115200);

    while (LED.begin(LED.e8Bit) != 0) {
        Serial.println("Failed to initialize the chip , please confirm the chip connection!");
        delay(1000);
    }

    LED.setDisplayArea(1, 2, 3, 4, 5, 6, 7, 8); // make all 8 digits available

    LTC_string[11] = '\0'; // terminate the string
}

void loop()
{
    count_timecode(h, m, s, f);

    update(LTC_string);

    Serial.println(LTC_string);
    print_to_display(LTC_string);

    increment_frames(); // LAST

    delay(1000 / FRAME_RATE); // delay the length of a 24fps frame
}

/* Clamp and increment time values
[h]ours, [m]inutes, [s]econds, [f]rames */
void count_timecode(uint8_t& h, uint8_t& m, uint8_t& s, uint8_t& f)
{
    if (f > FRAME_RATE - 1) {
        f = 0;
        s++;
    }

    if (s > 59) {
        s = 0;
        m++;
    }

    if (m > 59) {
        m = 0;
        h++;
    }

    if (h > 23) {
        h = 0;
    }
}

/* recreate the timecode string to print
                    _ _ . _ _ . _ _ . _ _ \0
  char[12] index:   0 1 2 3 4 5 6 7 8 9 0 1
*/
void update(char* LTC_string)
{
    sprintf(LTC_string, "%02d.%02d.%02d.%02d", h, m, s, f);
}

void print_to_display(char* LTC_string)
{
    LED.print(
        &LTC_string[0], &LTC_string[1], // hours, ones place + '.'
        &LTC_string[3], &LTC_string[4], // minutes, "
        &LTC_string[6], &LTC_string[7], // seconds, "
        &LTC_string[9], &LTC_string[10] // frames, "
    );
}