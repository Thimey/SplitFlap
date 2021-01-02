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

void ISR_Sensor()
{
    splitFlapArray.ISR_Sensor();
}

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

    if(!client.connected()){
        Serial.println("AWS IoT Timeout!");
        return;
    }

    // Subscribe to screen update topic
    client.subscribe(WORD_SUB_TOPIC);

    Serial.println("AWS IoT Connected!");
}

void messageHandler(String &topic, String &payload) {
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);

    const char* word = doc["word"];

    splitFlapArray.setWord(word);
}

void setup() {
    Serial.begin(9600);

    pinMode(SR_LATCH_PIN, OUTPUT);
    pinMode(SR_CLOCK_PIN, OUTPUT);
    pinMode(SR_DATA_PIN, OUTPUT);

    pinMode(DIR_PIN, OUTPUT);
    pinMode(SENSOR_PIN, INPUT);

    digitalWrite(DIR_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), ISR_Sensor, FALLING);

    // Connect to the configured WiFi
    connectWiFi();

    // Configure WiFiClientSecure to use the AWS IoT device credentials
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    // Connect to the MQTT broker to AWS endpoint for Thing
    client.begin(AWS_IOT_ENDPOINT, 8883, net);

    // Create a message handler
    client.onMessage(messageHandler);

    // Connect device to AWS iot
    connectAWS();

    // TODO: Uncomment this when sensors installed
    // Ensure all split flaps start from blank
    // splitFlapArray.resetFlaps();
}

void loop() {
    // Maintain a connection to the server
    client.loop();

    if (!client.connected()) {
        Serial.println(F("Lost connection, reconnecting..."));
        connectAWS();
    }

    delay(1000);

    splitFlapArray.loop();
}