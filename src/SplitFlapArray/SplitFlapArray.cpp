
#include <Arduino.h>

#include "config.h"
#include "SplitFlap/SplitFlap.h"
#include "SplitFlapArray.h"

void printBits(byte myByte){
 for(byte mask = 0x80; mask; mask >>= 1){
   if(mask  & myByte)
       Serial.print('1');
   else
       Serial.print('0');
 }
 Serial.println("");
}

SplitFlapArray::SplitFlapArray() :
    characterDisplays(sizeof(String), MAX_CHARACTER_DISPLAY_QUEUE, FIFO)
    , pauseQueueTime(DEFAULT_PAUSE_MS)
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
    delayMicroseconds(SplitFlapArray::stepDelayMicro);
    SplitFlapArray::shiftOutSteps(0);
    delayMicroseconds(SplitFlapArray::stepDelayMicro);
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

void SplitFlapArray::printSensorInput()
{
    printBits(SplitFlapArray::getSensorInput());
}

void SplitFlapArray::stepAllSensorsOff()
{
    byte sensorInput = SplitFlapArray::getSensorInput();

    // Step through flaps, until all sensors are off
    // TODO: Dynamically compute binary value for available flaps
    while (sensorInput != B11111110)
    {
        SplitFlapArray::stepSplitFlapArrayOnce(~sensorInput);
        sensorInput = SplitFlapArray::getSensorInput();
    }
}

void SplitFlapArray::resetFlaps()
{
    SplitFlapArray::enableMotors();

    // Since hall effect sensors cover a range of flaps and we want only the first flap
    // Step all flaps off motors
    SplitFlapArray::stepAllSensorsOff();

    byte sensorInput = SplitFlapArray::getSensorInput();

    // Step through flaps, till all sensors triggered.
    while (sensorInput != 0) {
        SplitFlapArray::stepSplitFlapArrayOnce(sensorInput);
        sensorInput = SplitFlapArray::getSensorInput();
    }

    // Reset the flap targets for each splitFlap
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        SplitFlapArray::splitFlaps[i].reset();
    }
}

void SplitFlapArray::stepAll(int rotations)
{
  SplitFlapArray::resetFlaps();

  int completedRotations = 0;

  while (completedRotations < rotations) {
    for (int i = 0; i < NUMBER_OF_FLAPS; ++i) {
      SplitFlapArray::stepSplitFlapArrayOnce(B11111110);
    }
    completedRotations = completedRotations + 1;
  }
}

void SplitFlapArray::stepSplitFlapsOnce(bool flapsToStep[NUMBER_OF_SPLIT_FLAPS])
{
  bool splitFlapsToStep[NUMBER_OF_SPLIT_FLAPS];

  // Update the internal state of each split flap that requires stepping
  for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i)
  {
      if (flapsToStep[i]) {
        bool isFlapAtTarget = SplitFlapArray::splitFlaps[i].step();

        splitFlapsToStep[i] = !isFlapAtTarget;
      }
  }

  const uint8_t shiftInput = SplitFlapArray::toShiftInput(splitFlapsToStep);

  SplitFlapArray::stepSplitFlapArrayOnce(shiftInput);
}

void SplitFlapArray::setCharacterDisplay(String characters)
{
    for (int i = 0; i < characters.length(); ++i) {
        char character = characters[i];
        if (character && i < NUMBER_OF_SPLIT_FLAPS) {
            SplitFlapArray::splitFlaps[i].setFlapTarget(character);
        }
    }
}

void SplitFlapArray::queueCharacterDisplay(String characters, int stepDelay = DEFAULT_PULSE_DELAY)
{
    SplitFlapArray::stepDelayMicro = stepDelay;
    SplitFlapArray::characterDisplays.push(&characters);
}

void SplitFlapArray::stepToCurrentCharacterDisplay()
{
    SplitFlapArray::enableMotors();

    while (!SplitFlapArray::hasSplitFlapArrayReachedTarget()) {
        SplitFlapArray::stepSplitFlapArray();
    }
}

void SplitFlapArray::loop()
{
    if (!SplitFlapArray::hasSplitFlapArrayReachedTarget()) {
        SplitFlapArray::stepToCurrentCharacterDisplay();
    } else if (!SplitFlapArray::characterDisplays.isEmpty()) {
        String characters;
        SplitFlapArray::characterDisplays.pop(&characters);
        SplitFlapArray::setCharacterDisplay(characters);
    }
}