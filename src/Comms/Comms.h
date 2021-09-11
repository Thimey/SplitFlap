#pragma once
#define _TASK_OO_CALLBACKS

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>

typedef void (*MessageHandler)(String &topic, String &payload);

void commsInitialise(MessageHandler messageHandler);
void commsLoop();
