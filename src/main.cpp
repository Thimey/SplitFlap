// #define _TASK_OO_CALLBACKS

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <TaskScheduler.h>

#include "Comms/Comms.h"
#include "SplitFlap/SplitFlap.h"
#include "SplitFlapArray/SplitFlapArray.h"
#include "config.h"
#include "secrets.h"

Scheduler taskRunner;

Comms comms;
SplitFlapArray splitFlapArray(&taskRunner);

void messageHandler(String &topic, String &payload) {
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
      Serial.println("Message received: ");
      Serial.print(topic);

    if (topic == DISPLAY_SUB_TOPIC) {
        JsonArray characterDisplays = doc["characterDisplays"].as<JsonArray>();
        int stepDelay = doc["stepDelay"].as<int>();
        for (JsonVariant characterDisplay : characterDisplays) {
            splitFlapArray.queueCharacterDisplay(characterDisplay.as<String>(), stepDelay);
        }
    } else if (topic == RESET_SUB_TOPIC) {
        splitFlapArray.resetFlaps();
    } else if (topic == DISABLE_MOTORS_TOPIC) {
        splitFlapArray.disableMotors();
    } else if (topic == ENABLE_MOTORS_TOPIC) {
        splitFlapArray.enableMotors();
    } else if (topic == GET_SENSOR_INPUT_TOPIC) {
        splitFlapArray.printSensorInput();
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

    comms.initialise(messageHandler);
}

void loop() {
    taskRunner.execute();
}