#pragma once

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
        void enableMotors();
        void disableMotors();

    public:
        SplitFlapArray();

        void setWord(const char* word);
        void resetFlaps();
        void ISR_Sensor();

        void loop();
};