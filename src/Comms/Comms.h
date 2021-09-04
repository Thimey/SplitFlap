#pragma once
#define _TASK_OO_CALLBACKS

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>

typedef void (*MessageHandler)(String &topic, String &payload);

class Comms : public Task {
    public:
        Comms(Scheduler* taskRunner);
        void initialise(MessageHandler messageHandler);
        bool Callback();

};
