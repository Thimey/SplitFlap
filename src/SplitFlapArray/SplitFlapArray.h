#pragma once
#define _TASK_OO_CALLBACKS

#include <cppQueue.h>
#include "config.h"
#include "SplitFlap/SplitFlap.h"
#include <TaskSchedulerDeclarations.h>

class SplitFlapArray : public Task {
    private:
        SplitFlap splitFlaps[NUMBER_OF_SPLIT_FLAPS];
        cppQueue characterDisplays;
        Scheduler* taskRunner;
        int pauseQueueTime;
        int stepDelayMicro;

        uint8_t boolArrayToBits(bool shouldStepValues[NUMBER_OF_SPLIT_FLAPS]);
        void shiftOutSteps(uint8_t shiftInput);
        void stepSplitFlapArrayOnce(uint8_t shiftInput);
        void stepSplitFlapArray();
        void stepSingleSplitFlap(int flapIndexToStep);
        void stepAllSensorsOff();
        bool hasSplitFlapArrayReachedTarget();
        void setCharacterDisplay(String characters);
        void scheduleTasks(String characters);
        void stepToCurrentCharacterDisplay();
        byte getSensorInput();


    public:
        SplitFlapArray(Scheduler* taskRunner);

        boolean Callback();
        void printSensorInput();
        void queueCharacterDisplay(String characters, int stepDelay);
        void resetFlaps();
        void enableMotors();
        void disableMotors();
        void stepAll(int rotations);
        void stepSplitFlapsOnce(bool flapsToStep[NUMBER_OF_SPLIT_FLAPS]);
        void loop();
};