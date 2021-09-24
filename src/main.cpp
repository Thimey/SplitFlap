#define _TASK_LTS_POINTER
#define _TASK_MICRO_RES
#define _TASK_STATUS_REQUEST

#include <Arduino.h>
#include <ArduinoJson.h>
#include <TaskScheduler.h>
#include <cppQueue.h>

#include "Comms/Comms.h"
#include "config.h"
#include "helpers.h"
#include "secrets.h"

void stepReadySplitFlaps();
void displayNextTask();
bool onDisplayTaskEnable();

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

StatusRequest displayStatus;

// Setup scheduler
Scheduler taskRunner;
// commsTask: Maintains connection to AWS
Task commsTask(500000, TASK_FOREVER, &commsLoop, &taskRunner, true);
// stepperTask: steps motors of any splitFlap that requires stepping
Task* stepperTask;
// splitFlapTasks: Array of splitFlapTasks that control the timing of when a unit should step
Task* splitFlapTasks[NUMBER_OF_SPLIT_FLAPS];
// displayTask: Runs queued displays
Task displayTask(&displayNextTask, &taskRunner);

cppQueue queuedDisplays(sizeof(displayData*), MAX_CHARACTER_DISPLAY_QUEUE, FIFO);
cppQueue displayTasksToDelete(sizeof(Task*), MAX_CHARACTER_DISPLAY_QUEUE, FIFO);



void enableMotors() {
    digitalWrite(ENABLE_PIN, LOW);
}

void disableMotors() {
    digitalWrite(ENABLE_PIN, HIGH);
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

void stepSplitFlap() {
    // Mark splitFlap as ready to step
    splitFlapData& data = *((splitFlapData*) taskRunner.currentLts());
    data.readyToStep = true;
}

void onSplitFlapDisable() {
    // Once a splitFlap has reached it's target, send a signal to displayStatus
    displayStatus.signal();
}

void stepReadySplitFlaps() {
    bool splitFlapsToStep[NUMBER_OF_SPLIT_FLAPS];
    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        Task* splitFlapTask = splitFlapTasks[i];
        splitFlapData& data = *((splitFlapData*) splitFlapTask->getLtsPointer());
        splitFlapsToStep[i] = splitFlapTask->isEnabled() && data.readyToStep;

        // Reset all splitFlaps
        data.readyToStep = false;
    }

    // Step the splitFlaps
    uint8_t shiftInput = boolArrayToBits(splitFlapsToStep);
    stepSplitFlapArrayOnce(shiftInput);
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

void showDisplay(displayData* data) {
    String characters = data->characters;
    int scheduledSplitFlapCount = 0;

    // Schedule the splitFlap tasks for characters
    for (int i = 0; i < characters.length(); ++i) {
        char nextCharacter = characters[i];

        if (nextCharacter && i < NUMBER_OF_SPLIT_FLAPS) {
            Task* splitFlapTask = splitFlapTasks[i];
            splitFlapData& data = *((splitFlapData*) splitFlapTask->getLtsPointer());

            uint8_t currentCharacter = data.lastCharacter;
            int stepsToNextCharacter = getStepsToNextCharacter(currentCharacter, nextCharacter);

            if (stepsToNextCharacter > 0) {
                scheduledSplitFlapCount += 1;
                data.lastCharacter = nextCharacter; // Move this to onDisable?
                data.reachedTarget = false;
                data.readyToStep = false;

                splitFlapTask->setIterations(stepsToNextCharacter);
                splitFlapTask->enable();
            }
        }
    }

    // Set displayTask to wait for completion of this display
    displayStatus.setWaiting(scheduledSplitFlapCount);
    displayTask.waitForDelayed(&displayStatus, DEFAULT_DISPLAY_PAUSE_US);

    stepperTask->enable();

    // Clean up display data
    delete data;
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

void displayNextTask() {
    if (!queuedDisplays.isEmpty()) {
        displayData * data;
        queuedDisplays.pop(&data);

        showDisplay(data);
    }
}

void queueDisplay(String characters, int stepDelay = DEFAULT_PULSE_DELAY) {
    displayData* data = new displayData(characters);

    bool availableToShowDisplay = queuedDisplays.isEmpty() && displayStatus.completed();
    if (availableToShowDisplay) {
        showDisplay(data);
    } else {
        queuedDisplays.push(&data);
    }
}

// void startClock(String initialDisplay) {
//     // Hour 2
//     Task* hour2Task = splitFlapTasks[0];
//     // Hour 1
//     Task* hour1Task = splitFlapTasks[1];
//     // Minute 2
//     Task* minute2Task = splitFlapTasks[3];
//     // Minute 1
//     Task* minute1Task = splitFlapTasks[4];
//     // Second 2
//     Task* second2Task = splitFlapTasks[6];
//     // Second 1
//     Task* second1Task = splitFlapTasks[7];
// }

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

    // Ensure splitFlaps rotate correct direction
    digitalWrite(DIR_PIN, HIGH);

    commsInitialise(messageHandler);

    for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
        // Initialise splitFlap task
        Task *t = new Task(STEP_SPEED_US, TASK_ONCE, &stepSplitFlap, &taskRunner, false, NULL, &onSplitFlapDisable);
        // Initialise local data for splitFlap task
        splitFlapData* data = new splitFlapData(i);
        t->setLtsPointer(data);

        splitFlapTasks[i] = t;
    }

    stepperTask = new Task(STEP_SPEED_US / 2, TASK_FOREVER, &stepReadySplitFlaps, &taskRunner, false);
}

void loop() {
    taskRunner.execute();
}
