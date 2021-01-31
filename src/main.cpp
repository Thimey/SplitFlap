#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"

#include "SplitFlap/SplitFlap.h"
#include "SplitFlapArray/SplitFlapArray.h"
#include "config.h"
#include "secrets.h"

// WiFi client
WiFiClientSecure net = WiFiClientSecure();
// MQTT client
MQTTClient client = MQTTClient(256);
// Split flap Array
SplitFlapArray splitFlapArray = SplitFlapArray();

void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.print(".");
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

    Serial.println("AWS IoT Connected!");
}

void messageHandler(String &topic, String &payload) {
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);

    if (topic == DISPLAY_SUB_TOPIC) {
        JsonArray characterDisplays = doc["characterDisplays"].as<JsonArray>();
        for (JsonVariant characterDisplay : characterDisplays) {
            splitFlapArray.queueCharacterDisplay(characterDisplay.as<String>());
        }
    } else if (topic == RESET_SUB_TOPIC) {
        splitFlapArray.resetFlaps();
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

    digitalWrite(DIR_PIN, LOW);

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

    // Ensure all split flaps start from blank
    splitFlapArray.resetFlaps();
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