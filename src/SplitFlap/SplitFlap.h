#pragma once

class SplitFlap {
    private:

        int currentFlapIndex;
        int flapTargetIndex;
        bool resetting;

        int getFlapIndex(uint8_t flapCharacter);
        int getFlapsToRotate(int desiredIndex);
        int getStepsToRotate(int flapsToRotate);
        void incrementFlapIndex();

    public:
        SplitFlap();
        SplitFlap(String name);

        String name;
        int stepsToTarget;

        boolean isAtFlapTarget();

        void setFlapTarget(uint8_t flapCharacter);
        void reset();
        void stopReset();
        bool isResetting();
        bool step();
};
