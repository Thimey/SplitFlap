#include <Arduino.h>

#include "config.h"
#include "utils.h"

uint8_t utils::toShiftInput(bool shouldStepValues[NUMBER_OF_SPLIT_FLAPS])
{
    uint8_t shiftInput = 0;
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
    if (shouldStepValues[i]) {
        shiftInput |= 1 << (MAX_SPLIT_FLAPS - 1 - i);
    }
    }

    return shiftInput;
}