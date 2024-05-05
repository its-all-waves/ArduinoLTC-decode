#include "Dispatcher.h"
#include "EventData.h"

Dispatcher* dispatcher = Dispatcher::get();

EventData data;

dispatcher->subscribe(data);