#ifndef Event_H
#define Event_H

enum ReaderEvent {
    STARTED_SYNC = 100,
    LOST_SYNC,
    READ_NEW_FRAME,
    READ_NEW_SECOND
};

enum ClapperEvent {
    OPENED_CLAPPER = 200,
    CLOSED_CLAPPER,
};

// typedef enum Event {
//     STARTED_SYNC,
//     LOST_SYNC,
//     OPENED_CLAPPER,
//     CLOSED_CLAPPER,
//     READ_NEW_SECOND,
//     READ_NEW_FRAME,
// };

#endif // Event_H