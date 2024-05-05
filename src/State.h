#ifndef States_H
#define States_H

enum ReaderState {
    NO_SYNC = 1000,
    SYNC
};

enum ClapperState {
    OPEN = 2000,
    JUST_CLOSED,
    CLOSED
};

struct State {
    ReaderState reader = NO_SYNC;
    ClapperState clapper = CLOSED;
};

#endif // states_H
