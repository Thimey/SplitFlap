#pragma once

#include <cppQueue.h>
#include "config.h"
#include "SplitFlap/SplitFlap.h"

class SplitFlapArray {
    private:
        SplitFlap splitFlaps[NUMBER_OF_SPLIT_FLAPS];

        uint8_t toShiftInput(bool shouldStepValues[NUMBER_OF_SPLIT_FLAPS]);
        void shiftOutSteps(uint8_t shiftInput);
        void stepSplitFlapArrayOnce(uint8_t shiftInput);
        void stepSplitFlapArray();
        void stepSingleSplitFlap(int flapIndexToStep);
        bool hasSplitFlapArrayReachedTarget();
        void setCharacterDisplay(const char* word);
        void stepToCurrentCharacterDisplay();
        void enableMotors();
        void disableMotors();
        byte getSensorInput();
        cppQueue words;

    public:
        SplitFlapArray();

        void queueCharacterDisplay(const char* word);
        void resetFlaps();
        void loop();
};