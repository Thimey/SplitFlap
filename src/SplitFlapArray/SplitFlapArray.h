#pragma once

#include <cppQueue.h>
#include "config.h"
#include "SplitFlap/SplitFlap.h"

class SplitFlapArray {
    private:
        SplitFlap splitFlaps[NUMBER_OF_SPLIT_FLAPS];
        cppQueue characterDisplays;
        int pauseQueueTime;
        int stepDelayMicro;

        uint8_t toShiftInput(bool shouldStepValues[NUMBER_OF_SPLIT_FLAPS]);
        void shiftOutSteps(uint8_t shiftInput);
        void stepSplitFlapArrayOnce(uint8_t shiftInput);
        void stepSplitFlapArray();
        void stepSingleSplitFlap(int flapIndexToStep);
        void stepAllSensorsOff();
        bool hasSplitFlapArrayReachedTarget();
        void setCharacterDisplay(String characters);
        void stepToCurrentCharacterDisplay();
        byte getSensorInput();


    public:
        SplitFlapArray();

        void printSensorInput();
        void queueCharacterDisplay(String characters, int stepDelay);
        void resetFlaps();
        void enableMotors();
        void disableMotors();
        void stepAll(int rotations);
        void stepSplitFlapsOnce(bool flapsToStep[NUMBER_OF_SPLIT_FLAPS]);
        void loop();
};