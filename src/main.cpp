#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <TaskScheduler.h>


#include "SplitFlap/SplitFlap.h"
#include "SplitFlapArray/SplitFlapArray.h"
#include "config.h"
#include "secrets.h"


Scheduler taskRunner;

void stepDefaultSpeedDisplay();


Task defaultSpeedDisplay(DEFAULT_PAUSE_MS, TASK_FOREVER, &stepDefaultSpeedDisplay, &taskRunner, true);


// WiFi client
WiFiClientSecure net = WiFiClientSecure();
// MQTT client
MQTTClient client = MQTTClient(256);
// Split flap Array
SplitFlapArray splitFlapArray = SplitFlapArray();


void stepDefaultSpeedDisplay()
{
  bool allStepped = SplitFlapArray::stepDefaultSpeedDisplays();

  if (allStepped) {
    // Stop task
  }
}

void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED){
        delay(1000);
        Serial.print(".");
        // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
}

void connectAWS() {
    Serial.print("Connecting to AWS IOT");

    while (!client.connect(THINGNAME)) {
        Serial.print(".");
        delay(1000);
    }

    if (!client.connected()){
        Serial.println("AWS IoT Timeout!");
        return;
    }

    // Subscribe to topics
    client.subscribe(DISPLAY_SUB_TOPIC);
    client.subscribe(RESET_SUB_TOPIC);
    client.subscribe(DISABLE_MOTORS_TOPIC);
    client.subscribe(ENABLE_MOTORS_TOPIC);
    client.subscribe(GET_SENSOR_INPUT_TOPIC);

    Serial.println("AWS IoT Connected!");
}

void messageHandler(String &topic, String &payload) {
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);

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

    // Connect to the configured WiFi
    connectWiFi();

    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    // Connect to the MQTT broker to AWS endpoint for Thing
    client.begin(AWS_IOT_ENDPOINT, 8883, net);

    // Attach message handler to MQTT broker
    client.onMessage(messageHandler);

    // Connect device to AWS iot
    connectAWS();
}

void loop() {
    // Maintain a connection to the server
    client.loop();

    if (!client.connected()) {
        Serial.println(F("Lost connection, reconnecting..."));
        connectAWS();
    }

    delay(500);

    splitFlapArray.loop();
}