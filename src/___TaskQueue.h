#ifndef TaskQueue_H
#define TaskQueue_H

#include "Dispatcher.h"
#include "EventData.h"

typedef struct Task {
    int event;
    EventData data;

    Task()
    {
        this->event = -1;
        this->data = EventData();
    }
};

#define MAX_TASKS_LEN 24

Task event_queue[MAX_TASKS];

int _next_task_index = 0;

void append_to_event_queue(Task queue[], Task task)
{
    event_queue[_next_task_index] = task;
    _next_task_index++;
}

int _next_pop_index = -1;

Task _pop(Task queue[])
{
    if (_next_task_index == 0) {
        Serial.println("There were no tasks to pop().");
        _next_pop_index = -1;
        return Task();
    }

    _next_pop_index++; // increment before use since init'd to -1
    Task task = queue[_next_pop_index];
    queue[_next_pop_index] = Task(); // "null" the task to simulate removal
    return task;
}

int _last_task_index = 0;

void run_tasks(Task queue[])
{
    while (_next_pop_index != -1) {
        Task task = _pop(queue);
        if (task.event == -1) {
            // this is an empty task (was already pop'd)
            Serial.print("Do we ever make it here?");
            continue;
        }
        // have the task, what do i do with it?!?!?
        Dispatcher dispatcher = *Dispatcher::get();
        dispatcher.post_event()
    }

    // clear

    // reset
    _next_pop_index = 0;
    _last_task_index = 0;
}

/*
typedef struct TaskQueue {
private:
    static TaskQueue* instance;

    TaskQueue() { }

public:
    static TaskQueue* get() { return instance == NULL ? new TaskQueue() : instance; }

    Task array[MAX_TASKS_LEN];

    boolean append(Task task)
    {
        array[next_task_index] = task;
        next_task_index++;
        return true;
    }

    void run()
    {
        int i;
        while (last_used_pop_index != -1) {
            Task task = pop();
            int event_enum = task.event;
            EventData data = task.data;
        }

        clear_and_reset();
    }

private:
    int next_task_index = 0;

    int last_used_pop_index = -1; // increments with each pop
    Task pop()
    {
        if (next_task_index == 0) {
            Serial.println("There were no tasks to pop().");
            last_used_pop_index = -1;
            return Task();
        }
        Task task = array[++last_used_pop_index]; // increment before use

        return task;
    }

    void clear_and_reset()
    {
        // reinitialize entire task queue
        for (int i; i < MAX_TASKS_LEN; i++) {
            array[i] = Task();
        }
        last_used_pop_index = -1;
        next_task_index = 0;
    }
};
 */

#endif // TaskQueue_H