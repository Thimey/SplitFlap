#pragma once

#include <cppQueue.h>
#include "config.h"
#include "SplitFlap/SplitFlap.h"

class SplitFlapArray {
    private:
        SplitFlap splitFlaps[NUMBER_OF_SPLIT_FLAPS];
        cppQueue characterDisplays;
        int pauseQueueTime;

        uint8_t toShiftInput(bool shouldStepValues[NUMBER_OF_SPLIT_FLAPS]);
        void shiftOutSteps(uint8_t shiftInput);
        void stepSplitFlapArrayOnce(uint8_t shiftInput);
        void stepSplitFlapArray();
        void stepSingleSplitFlap(int flapIndexToStep);
        bool hasSplitFlapArrayReachedTarget();
        void setCharacterDisplay(String characters);
        void stepToCurrentCharacterDisplay();
        void enableMotors();
        void disableMotors();
        byte getSensorInput();

    public:
        SplitFlapArray();

        void queueCharacterDisplay(String characters);
        void resetFlaps();
        void loop();
};