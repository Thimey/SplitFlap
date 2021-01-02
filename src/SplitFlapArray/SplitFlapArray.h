#pragma once

#include "config.h"
#include "SplitFlap/SplitFlap.h"

class SplitFlapArray {
    private:
        SplitFlap splitFlaps[NUMBER_OF_SPLIT_FLAPS];

        void shiftOutSteps(uint8_t shiftInput);
        void stepSplitFlapArrayOnce(uint8_t shiftInput);
        void stepSplitFlapArray();
        bool hasSplitFlapArrayReachedTarget();

    public:
        SplitFlapArray();

        void setWord(const char* word);
        void resetFlaps();
        void ISR_Sensor();

        void loop();
};