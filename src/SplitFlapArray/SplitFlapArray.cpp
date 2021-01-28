
#include <Arduino.h>

#include "config.h"
#include "SplitFlap/SplitFlap.h"
#include "SplitFlapArray.h"

SplitFlapArray::SplitFlapArray() :
    words(NUMBER_OF_SPLIT_FLAPS * sizeof(char), MAX_WORD_COUNT, FIFO)
{
    // Initialise SplitFlap objects
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
    digitalWrite(SR_DRIVER_LATCH_PIN, LOW);
    // Write the step data to register
    shiftOut(SR_DRIVER_DATA_PIN, SR_DRIVER_CLOCK_PIN, LSBFIRST, shiftInput);
    // Release the data
    digitalWrite(SR_DRIVER_LATCH_PIN, HIGH);
}

void SplitFlapArray::stepSplitFlapArrayOnce(uint8_t shiftInput)
{
    SplitFlapArray::shiftOutSteps(shiftInput);
    delayMicroseconds(PULSE_DELAY);
    SplitFlapArray::shiftOutSteps(0);
    delayMicroseconds(PULSE_DELAY);
}

void SplitFlapArray::stepSingleSplitFlap(int flapIndexToStep)
{
    bool splitFlapsToStep[NUMBER_OF_SPLIT_FLAPS];
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        splitFlapsToStep[i] = i == flapIndexToStep;
    }

    const uint8_t shiftInput = SplitFlapArray::toShiftInput(splitFlapsToStep);

    SplitFlapArray::stepSplitFlapArrayOnce(shiftInput);
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

void SplitFlapArray::enableMotors()
{
    digitalWrite(ENABLE_PIN, LOW);
}

void SplitFlapArray::disableMotors()
{
    // Cuts current to motors
    digitalWrite(ENABLE_PIN, HIGH);
}

byte SplitFlapArray::getSensorInput()
{
    // Write pulse to load pin
    digitalWrite(SR_SENSOR_LOAD_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SR_SENSOR_LOAD_PIN, HIGH);
    delayMicroseconds(5);

    // Get data from 74HC165
    digitalWrite(SR_SENSOR_CLOCK_PIN, LOW);
    digitalWrite(SR_SENSOR_CLOCK_ENABLE_PIN, LOW);
    byte sensorInput = shiftIn(SR_SENSOR_DATA_PIN, SR_SENSOR_CLOCK_PIN, LSBFIRST);
    digitalWrite(SR_SENSOR_CLOCK_ENABLE_PIN, HIGH);

    // Since bits are inverted from shift register, invert them for actual sensor values
    return ~sensorInput;
}

void SplitFlapArray::resetFlaps()
{
    SplitFlapArray::enableMotors();

    // Reset the flap targets for each splitFlap
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        SplitFlapArray::splitFlaps[i].reset();
    }

    const int MAX_STEPS_TO_RESET = 2 * STEPS_PER_REVOLUTION;
    int stepCount = 0;
    byte sensorInput = SplitFlapArray::getSensorInput();

    // Step through flaps, till sensor triggered.
    // Stop resetting if two revolutions have stepped - this means something is broken.
    while (sensorInput != 0 && stepCount < MAX_STEPS_TO_RESET) {
        SplitFlapArray::stepSplitFlapArrayOnce(sensorInput);
        sensorInput = SplitFlapArray::getSensorInput();
        ++stepCount;
    }

    SplitFlapArray::disableMotors();
}

void SplitFlapArray::setCharacterDisplay(const char* word)
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

void SplitFlapArray::queueCharacterDisplay(const char* word)
{
    SplitFlapArray::words.push(word);
}

void SplitFlapArray::stepToCurrentCharacterDisplay()
{
    SplitFlapArray::enableMotors();

    while (!SplitFlapArray::hasSplitFlapArrayReachedTarget()) {
        SplitFlapArray::stepSplitFlapArray();
    }

    SplitFlapArray::disableMotors();
}

void SplitFlapArray::loop()
{
    if (!SplitFlapArray::hasSplitFlapArrayReachedTarget()) {
        SplitFlapArray::stepToCurrentCharacterDisplay();
    } else if (!SplitFlapArray::words.isEmpty()) {
        char* word;
        SplitFlapArray::words.pop(&word);
        SplitFlapArray::setCharacterDisplay(word);
    }
}