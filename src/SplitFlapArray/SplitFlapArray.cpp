
#include <Arduino.h>

#include "config.h"
#include "SplitFlap/SplitFlap.h"
#include "SplitFlapArray.h"

SplitFlapArray::SplitFlapArray()
{
  for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
    String name = "splitFlap" + i;
    SplitFlapArray::splitFlaps[i] = SplitFlap(name);
  }
};

uint8_t SplitFlapArray::toShiftInput(bool shouldStepValues[NUMBER_OF_SPLIT_FLAPS])
{
    uint8_t shiftInput = 0;
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
    if (shouldStepValues[i]) {
        shiftInput |= 1 << (MAX_SPLIT_FLAPS - 1 - i);
    }
    }

    return shiftInput;
}

void SplitFlapArray::shiftOutSteps(uint8_t shiftInput)
{
    // Prevent data from being released
    digitalWrite(SR_LATCH_PIN, LOW);
    // Write the step data to register
    shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, LSBFIRST, shiftInput);
    // Release the data
    digitalWrite(SR_LATCH_PIN, HIGH);
}

void SplitFlapArray::stepSplitFlapArrayOnce(uint8_t shiftInput)
{
    SplitFlapArray::shiftOutSteps(shiftInput);
    delayMicroseconds(PULSE_DELAY);
    SplitFlapArray::shiftOutSteps(0);
    delayMicroseconds(PULSE_DELAY);
}

void SplitFlapArray::stepSplitFlapArray()
{
    bool splitFlapsToStep[NUMBER_OF_SPLIT_FLAPS];

    // Update the internal state of each split flap
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i)
    {
        bool isFlapAtTarget = SplitFlapArray::splitFlaps[i].step();

        splitFlapsToStep[i] = !isFlapAtTarget;
    }

    const uint8_t shiftInput = SplitFlapArray::toShiftInput(splitFlapsToStep);

    SplitFlapArray::stepSplitFlapArrayOnce(shiftInput);
}

bool SplitFlapArray::hasSplitFlapArrayReachedTarget()
{
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        if (!SplitFlapArray::splitFlaps[i].isAtFlapTarget()) {
            return false;
        }
    }

    return true;
}

void SplitFlapArray::ISR_Sensor()
{
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        if (SplitFlapArray::splitFlaps[i].isResetting()) {
            SplitFlapArray::splitFlaps[i].stopReset();
        }
    }
}

void SplitFlapArray::resetFlaps()
{
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        // Reset the flap targets for each splitFlap
        SplitFlapArray::splitFlaps[i].reset();

        // Step through flaps, till sensor triggered
        while(SplitFlapArray::splitFlaps[i].isResetting()) {
            stepSplitFlapArrayOnce(B11111111);
        }
    }
}

void SplitFlapArray::setWord(const char* word)
{
    int wordIndex = 0;
    char character = word[0];

    while (character != '\0') {
        character = word[wordIndex];
        if (character && wordIndex < NUMBER_OF_SPLIT_FLAPS) {
            SplitFlapArray::splitFlaps[wordIndex].setFlapTarget(character);
        }
        ++wordIndex;
    }
}

void SplitFlapArray::loop()
{
    while (!SplitFlapArray::hasSplitFlapArrayReachedTarget()) {
        SplitFlapArray::stepSplitFlapArray();
    }
}