#ifndef EventData_H
#define EventData_H

#include "State.h"

/* All possible data types for an event's data */
typedef union EventData {
    // type: string resembling "12.34.56.23"
    char ltc_string[12];

    EventData() { strcpy(ltc_string, "--.--.--.--"); }
    EventData(const char* ltc) { strcpy(ltc_string, ltc); }

    // type: pair of ltc strings
    struct LTC {
        // --.--.--.-- = 11 chars + 1 for null char = 12
        char tc_string[12]; // null check using strcmp('\0')
        char ub_string[12];
    } LTC;

    EventData(const char* tc, const char* ub)
    {
        strcpy(LTC.tc_string, tc);
        strcpy(LTC.ub_string, ub);
    }

    // type:
    State state;

    EventData(const State& s)
        : state(s)
    {
    }
};

#endif // EventData_H