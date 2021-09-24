#pragma once

#include <Arduino.h>
#include <TaskSchedulerDeclarations.h>

typedef void (*MessageHandler)(String &topic, String &payload);

void commsInitialise(MessageHandler messageHandler);
void commsLoop();
