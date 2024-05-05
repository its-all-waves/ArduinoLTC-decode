#ifndef Dispatcher_H
#define Dispatcher_H

#include "Event.h"
#include "EventCallbacks.h"
#include "EventData.h"

#define MAX_SUBSCRIBERS_LEN 6

#define MAX_TASKS 6

typedef class Dispatcher {
private:
    EventCallbackList subscribers[MAX_SUBSCRIBERS_LEN];

    static Dispatcher instance;

    Dispatcher() { }

    class TaskQueue {
    private:
        uint8_t queue_len = 0;
        uint8_t pop_cursor = 0;

    public:
        typedef struct Task {
            int event;
            EventData data;

            Task()
            {
                this->event = -1;
                this->data = EventData();
            }

            Task(int event, EventData data)
                : event(event)
                , data(data)
            {
            }
        };
        Task tasks[MAX_TASKS];

        uint8_t get_len()
        {
            return queue_len;
        }

        boolean push(int event_enum, EventData data)
        {
            if (queue_len == MAX_TASKS) {
                Serial.println("Failed to add a task to the queue. Maximum tasks reached.");
                return false;
            }
            tasks[queue_len] = Task(event_enum, data);
            queue_len++;

            // Serial.println("PUSH TO QUEUE SUCCESSFUL!");
            return true;
        }

        Task pop()
        {
            if (queue_len == 0) {
                Serial.println("There were no tasks to pop().");
                pop_cursor = 0;
                return Task();
            }
            Task task = tasks[pop_cursor];
            queue_len--;
            pop_cursor = queue_len == 0
                ? 0
                : pop_cursor + 1;
            return task;
        }

        void clear_and_reset()
        {
            // reinitialize entire task queue
            for (int i; i < MAX_TASKS; i++) {
                tasks[i] = Task();
            }
            pop_cursor = 0;
            queue_len = 0;
        }
    };

    TaskQueue queue;

    int subscribers_len = 0;

    /* Add an EventCallbacks obj to subscribers. Returns false if an
    EventCallbacks obj with given Event already exists. */
    boolean add_to_subscribers(int event_enum)
    {
        // return if already exists
        if (index_of_subscribers(event_enum) >= 0) {
            return false;
        }
        subscribers[subscribers_len] = EventCallbackList(event_enum);
        subscribers_len++;
        return true;
    }

    /* Returns -1 if there is no EventCallbacks obj with the given Event, else
    the index of an EventCallbacks obj in subscribers. */
    int index_of_subscribers(int event_enum)
    {
        for (int i; i < MAX_SUBSCRIBERS_LEN; i++) {
            if (subscribers[i].event == event_enum) {
                return i;
            }
        }
        return -1;
    }

public:
    static Dispatcher& get() { return instance; }
    // static Dispatcher* get() { return instance == NULL ? new Dispatcher() : instance; }

    void subscribe(int event_enum, EventCallbackPtr callback)
    {
        // if event not in subscribers, add it
        int i = index_of_subscribers(event_enum);
        if (i < 0) {
            add_to_subscribers(event_enum);
        }
        // append the callback ptr to the EventCallbacks obj with given event
        subscribers[i].append(callback);
    }

    void queue_event(int event_enum, EventData data)
    {

        int i = index_of_subscribers(event_enum);
        if (i < 0) {
            // TODO: handle case
        }

        // queue up the corresponding callbacks obj for execution in the main thread
        queue.push(event_enum, data);

        // Serial.println("QUEUE_EVENT WAS COMPLETED!");
    }

    void flush_queue()
    {
        // loop over queue executing event's callbacks
        while (queue.get_len() > 0) {
            TaskQueue::Task task = queue.pop();

            int index = index_of_subscribers(task.event);
            EventCallbackList callback_list = subscribers[index];

            // int length = callback_list.get_len();
            Serial.print("length: ");
            Serial.println(callback_list.get_len());
            // Serial.println("FLUSH QUEUE WAS ACTUALLY called!");

            // execute every function in the callback list
            for (int i = 0; i < callback_list.get_len(); i++) {
                callback_list.callbacks[i](task.data);
            }
        }

        // queue.clear_and_reset();
    }

    uint8_t get_queue_len()
    {
        return queue.get_len();
    }
};

// Dispatcher* Dispatcher::instance = NULL;
Dispatcher Dispatcher::instance;

#endif // Dispatcher_H
