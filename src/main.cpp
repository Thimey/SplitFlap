#define _TASK_LTS_POINTER
#define _TASK_MICRO_RES

#include <Arduino.h>
#include <ArduinoJson.h>
#include <TaskScheduler.h>
#include <cppQueue.h>

#include "Comms/Comms.h"
#include "config.h"
#include "secrets.h"

void stepReadySplitFlaps();
void garbageCollection();

struct displayData {
    displayData(String characters) : characters(characters) {}
    String characters;
};
 struct splitFlapData {
    splitFlapData(int index) :
        index(index)
        , lastCharacter(' ')
        , readyToStep(false)
        , reachedTarget(true) {}
    int index;
    uint8_t lastCharacter;
    bool readyToStep;
    bool reachedTarget;
};

// Setup scheduler
Scheduler taskRunner;
// commsTask: Maintains connection to AWS
Task commsTask(500000, TASK_FOREVER, &commsLoop, &taskRunner, true);
// stepperTask: steps motors of any splitFlap that requires stepping
Task* stepperTask;
// splitFlapTasks: Array of splitFlapTasks that control the timing of when a unit should step
Task* splitFlapTasks[NUMBER_OF_SPLIT_FLAPS];
// garbageCollectionTask: Cleans up finished tasks on the heap
Task garbageCollectionTask(2000000, TASK_FOREVER, &garbageCollection, &taskRunner, true);

cppQueue queuedDisplays(sizeof(Task*), MAX_CHARACTER_DISPLAY_QUEUE, FIFO);
cppQueue displayTasksToDelete(sizeof(Task*), MAX_CHARACTER_DISPLAY_QUEUE, FIFO);

void printBits(byte myByte){
    for (byte mask = 0x80; mask; mask >>= 1){
        if (mask  & myByte) {
            Serial.print('1');
        }
        else {
            Serial.print('0');
        }
    }
    Serial.println("");
}

void shiftOutSteps(uint8_t shiftInput)
{
    // Prevent data from being released
    digitalWrite(SR_DRIVER_LATCH_PIN, LOW);
    // Write the step data to register
    shiftOut(SR_DRIVER_DATA_PIN, SR_DRIVER_CLOCK_PIN, LSBFIRST, shiftInput);
    // Release the data
    digitalWrite(SR_DRIVER_LATCH_PIN, HIGH);
}

void stepSplitFlapArrayOnce(uint8_t shiftInput)
{
    shiftOutSteps(shiftInput);
    delayMicroseconds(DEFAULT_PULSE_DELAY);
    shiftOutSteps(0);
    delayMicroseconds(DEFAULT_PULSE_DELAY);
}


uint8_t boolArrayToBits(bool shouldStepValues[NUMBER_OF_SPLIT_FLAPS])
{
    uint8_t shiftInput = 0;
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        if (shouldStepValues[i]) {
            shiftInput |= 1 << (MAX_SPLIT_FLAPS - 1 - i);
        }
    }

    return shiftInput;
}

void stepSplitFlap() {
    // Mark splitFlap as ready to step
    splitFlapData& data = *((splitFlapData*) taskRunner.currentLts());
    data.readyToStep = true;

    if (taskRunner.currentTask().isLastIteration()) {
        data.reachedTarget = true;
    }
}


void stepReadySplitFlaps() {
    // Check which splitFlap are ready to step
    bool splitFlapsToStep[NUMBER_OF_SPLIT_FLAPS];
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        Task* splitFlapTask = splitFlapTasks[i];
        splitFlapData& data = *((splitFlapData*) splitFlapTask->getLtsPointer());
        splitFlapsToStep[i] = splitFlapTask->isEnabled() && data.readyToStep;

        // Mark split flap as stepped
        data.readyToStep = false;

        if (data.reachedTarget) {
            splitFlapTask->disable();
        }
    }

    // Step the splitFlaps
    uint8_t shiftInput = boolArrayToBits(splitFlapsToStep);
    stepSplitFlapArrayOnce(shiftInput);

    // If display finished and others are queued, enable next display with delay
    bool displayFinished = shiftInput == 0;
    if (displayFinished && !queuedDisplays.isEmpty()) {
            Task* nextDisplay;
            queuedDisplays.pop(&nextDisplay);
            nextDisplay->enableDelayed(DEFAULT_DISPLAY_PAUSE_US);
    }
}

void enableMotors() {
    digitalWrite(ENABLE_PIN, LOW);
}

void disableMotors() {
    digitalWrite(ENABLE_PIN, HIGH);
}

int flapIndexFromCharacter(uint8_t flapCharacter)
{
    for (int i = 0; i < NUMBER_OF_FLAPS; ++i) {
        if (flaps[i] == flapCharacter) {
            return i;
        }
    }

    return 0;
}

int getStepsToNextCharacter(uint8_t currentCharacter, uint8_t nextCharacter) {
    int currentFlapIndex = flapIndexFromCharacter(currentCharacter);
    int nextFlapIndex = flapIndexFromCharacter(nextCharacter);
    int indexDiff = nextFlapIndex - currentFlapIndex;

    int flapsToNextCharacter = indexDiff < 0
        ? NUMBER_OF_FLAPS + indexDiff
        : indexDiff;

    return flapsToNextCharacter * (STEPS_PER_REVOLUTION / NUMBER_OF_FLAPS);
}


byte getSensorInput()
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

void stepAllSensorsOff() {
    byte sensorInput = getSensorInput();

    // Step through flaps, until all sensors are off
    // TODO: Dynamically compute binary value for available flaps
    while (sensorInput != B11111111)
    {
        stepSplitFlapArrayOnce(~sensorInput);
        sensorInput = getSensorInput();
    }
}

void onDisplayTaskDisable() {
    // Add finished display task to delete queue to be garbage collected
    Task* finishedDisplay = &taskRunner.currentTask();
    displayTasksToDelete.push(finishedDisplay);
}

void showDisplay() {
    // Get characters to display from task ls
    displayData& data = *((displayData*) taskRunner.currentLts());
    String characters = data.characters;

    // Schedule the splitFlap tasks for characters
    for (int i = 0; i < characters.length(); ++i) {
        char nextCharacter = characters[i];

        if (nextCharacter && i < NUMBER_OF_SPLIT_FLAPS) {
            Task* splitFlapTask = splitFlapTasks[i];
            splitFlapData& data = *((splitFlapData*) splitFlapTask->getLtsPointer());

            uint8_t currentCharacter = data.lastCharacter;
            int stepsToNextCharacter = getStepsToNextCharacter(currentCharacter, nextCharacter);

            if (stepsToNextCharacter > 0) {
                data.lastCharacter = nextCharacter; // Move this to onDisable?
                data.reachedTarget = false;
                data.readyToStep = false;

                splitFlapTask->setIterations(stepsToNextCharacter);
                splitFlapTask->enable();
            }
        }
    }

    // Can prob remove this
    stepperTask->enable();
}

void resetFlaps() {
    enableMotors();

    // Since hall effect sensors cover a range of flaps and we want only the first flap
    // Step all flaps off motors
    stepAllSensorsOff();

    byte sensorInput = getSensorInput();

    // Step through flaps, till all sensors triggered.
    while (sensorInput != 0) {
        stepSplitFlapArrayOnce(sensorInput);
        sensorInput = getSensorInput();
    }

    // Reset the characters for all splitFlap tasks
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        splitFlapData& data = *((splitFlapData*) splitFlapTasks[i]->getLtsPointer());
        data.lastCharacter = ' ';
    }
}

void queueDisplay(String characters, int stepDelay = DEFAULT_PULSE_DELAY) {
    // Create a disabled display task
    Task* displayTask = new Task(TASK_IMMEDIATE, TASK_ONCE, showDisplay, &taskRunner, false);
    // Add display date to task ls
    displayData* data = new displayData(characters);
    displayTask->setLtsPointer(data);

    // If nothing queued, enable task immediately
    if (queuedDisplays.isEmpty()) {
        displayTask->enable();
    } else {
        // Otherwise add to queue to be enabled later
        queuedDisplays.push(displayTask);
    }
}

void messageHandler(String &topic, String &payload) {
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
      Serial.println("Message received: ");
      Serial.print(topic);

    if (topic == DISPLAY_SUB_TOPIC) {
        JsonArray characterDisplays = doc["characterDisplays"].as<JsonArray>();
        int stepDelay = doc["stepDelay"].as<int>();
        for (JsonVariant characterDisplay : characterDisplays) {
            queueDisplay(characterDisplay.as<String>(), stepDelay);
        }
    } else if (topic == RESET_SUB_TOPIC) {
        resetFlaps();
    } else if (topic == DISABLE_MOTORS_TOPIC) {
        disableMotors();
    } else if (topic == ENABLE_MOTORS_TOPIC) {
        enableMotors();
    }
}

void setup() {
    Serial.begin(9600);

    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    pinMode(SR_DRIVER_LATCH_PIN, OUTPUT);
    pinMode(SR_DRIVER_CLOCK_PIN, OUTPUT);
    pinMode(SR_DRIVER_DATA_PIN, OUTPUT);

    pinMode(SR_SENSOR_LOAD_PIN, OUTPUT);
    pinMode(SR_SENSOR_CLOCK_ENABLE_PIN, OUTPUT);
    pinMode(SR_SENSOR_CLOCK_PIN, OUTPUT);
    pinMode(SR_SENSOR_DATA_PIN, INPUT);

    digitalWrite(DIR_PIN, HIGH);

    commsInitialise(messageHandler);

    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        // Initialise splitFlap task
        Task *t = new Task(STEP_SPEED, TASK_ONCE, stepSplitFlap, &taskRunner, false);
        // Initialise local data for splitFlap task
        splitFlapData* data = new splitFlapData(i);
        t->setLtsPointer(data);

        splitFlapTasks[i] = t;
    }

    stepperTask = new Task(STEP_SPEED / 2, TASK_FOREVER, &stepReadySplitFlaps, &taskRunner, false);
}

void garbageCollection() {
    if (!displayTasksToDelete.isEmpty()) {
        Task* displayTask;
        queuedDisplays.pop(&displayTask);

        // Clean up display data and task from heap
        delete displayTask->getLtsPointer();
        delete displayTask;
    }
}

void loop() {
    taskRunner.execute();
}
