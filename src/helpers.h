#pragma once

#include <Arduino.h>
#include "config.h"


void printBits(byte myByte){
    for (byte mask = 0x80; mask; mask >>= 1){
        if (mask  & myByte) {
            Serial.print('1');
        }
        else {
            Serial.print('0');
        }
    }
    Serial.println("");
}

uint8_t boolArrayToBits(bool shouldStepValues[NUMBER_OF_SPLIT_FLAPS])
{
    uint8_t shiftInput = 0;
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        if (shouldStepValues[i]) {
            shiftInput |= 1 << (MAX_SPLIT_FLAPS - 1 - i);
        }
    }

    return shiftInput;
}

int flapIndexFromCharacter(uint8_t flapCharacter)
{
    for (int i = 0; i < NUMBER_OF_FLAPS; ++i) {
        if (flaps[i] == flapCharacter) {
            return i;
        }
    }

    return 0;
}

int getStepsToNextCharacter(uint8_t currentCharacter, uint8_t nextCharacter, int calibrationSteps) {
    int currentFlapIndex = flapIndexFromCharacter(currentCharacter);
    int nextFlapIndex = flapIndexFromCharacter(nextCharacter);
    int indexDiff = nextFlapIndex - currentFlapIndex;

    int flapsToNextCharacter = indexDiff < 0
        ? NUMBER_OF_FLAPS + indexDiff
        : indexDiff;

    int flapsToNext = flapsToNextCharacter * (STEPS_PER_REVOLUTION / NUMBER_OF_FLAPS);

    return flapsToNext > 0 ? flapsToNext + calibrationSteps : 0;
}

char lastChar(String characters) {
    return characters.charAt(characters.length());
}

String timeDisplay(int num) {
    return num > 9 ? String(num) : "0" + String(num);
}

String makeDisplayFromSeconds(int totalSeconds) {
    int seconds = totalSeconds % 60;
    int minutes = (totalSeconds / 60) % 60;
    int hours = totalSeconds / (60 * 60);

    return timeDisplay(hours) + ":" + timeDisplay(minutes) + ":" + timeDisplay(seconds);
}
