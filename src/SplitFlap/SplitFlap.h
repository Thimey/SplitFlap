#pragma once

#include <TaskScheduler.h>
// #include <TaskSchedulerDeclarations.h>


class SplitFlap : public Task {
    private:
        String name;
        int currentFlapIndex;
        int flapTargetIndex;
        bool resetting;
        bool isReadyToStep;

        int getFlapIndex(uint8_t flapCharacter);
        int getFlapsToRotate(int desiredIndex);
        int getStepsToRotate(int flapsToRotate);
        void incrementFlapIndex();



    public:
        SplitFlap();
        SplitFlap(String name, Scheduler* taskRunner);

        int stepsToTarget;

        boolean Callback();

        boolean isAtFlapTarget();
        void readyToStep();
        void setFlapTarget(uint8_t flapCharacter);
        void scheduleCharacter(uint8_t flapCharacter, Scheduler* taskRunner);
        void reset();
        void stopReset();
        bool isResetting();
        bool step();
};
