#ifndef EventCallbacks_H
#define EventCallbacks_H

#include "Event.h"
#include "EventData.h"
#include "State.h"

typedef void (*EventCallbackPtr)(EventData data);

#define MAX_CALLBACK_ARR_LEN 10

typedef struct EventCallbackList {

    // enum "<type>Event"
    int event;

    // event handler funcs from other entities
    EventCallbackPtr callbacks[MAX_CALLBACK_ARR_LEN];

    EventCallbackList()
    {
        this->event = -1;

        // init array to nulls
        for (int i = 0; i < MAX_CALLBACK_ARR_LEN; i++) {
            this->callbacks[i] = NULL;
        }
    }

    EventCallbackList(int event_enum)
    {
        this->event = event_enum;
    }

    EventCallbackList(int event_enum, EventCallbackPtr callback)
    {
        this->event = event_enum;
        this->callbacks[len_callbacks] = callback;
    }

    // TODO: who is responsible for executing callbacks? this obj or the array that holds this obj?
    void execute_queue(EventData data)
    {
        for (int i; i < MAX_CALLBACK_ARR_LEN; i++) {
            callbacks[i](data);
        }
    }

    boolean append(EventCallbackPtr callback_ptr)
    {

        if (len_callbacks == MAX_CALLBACK_ARR_LEN) {
            Serial.println("Could not add a callback_ptr. Maximum exceeded.");
            return false;
        }
        callbacks[len_callbacks] = callback_ptr;
        len_callbacks++;
        // Serial.println("APPEND TO SUBSCRIBERS HAPPENED");
        return true;
    }

    int get_len()
    {
        return len_callbacks;
    }

private:
    uint8_t len_callbacks = 0;
};

#endif // EventCallbacks_H