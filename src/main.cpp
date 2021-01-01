#include <Arduino.h>
#include "SplitFlap/SplitFlap.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"

#include "config.h"
#include "secrets.h"

const int WORD_COUNT = 5;

const int SR_LATCH_PIN = 21;
const int SR_CLOCK_PIN = 23;
const int SR_DATA_PIN = 22;

const int SENSOR_PIN = 19;
const int DIR_PIN = 18;

// WiFi client
WiFiClientSecure net = WiFiClientSecure();
// MQTT client
MQTTClient client = MQTTClient(256);

SplitFlap splitflap1("splitFlap1");
SplitFlap splitflap2("splitFlap2");
SplitFlap splitflap3("splitFlap3");

SplitFlap splitFlapArray[NUMBER_OF_SPLIT_FLAPS] = {
  splitflap1,
  splitflap2,
  splitflap3
};

uint8_t flapDisplayCharacters[WORD_COUNT][NUMBER_OF_SPLIT_FLAPS] = {
  {'h', 'i', 'z'},
  {'a', '9', ','},
  {'3', 'f', 's'},
  {'h', 'i', 'a'},
  {'o', 's', '9'},
};


void ISR_SENSOR()
{
  splitFlapArray[0].stopReset();
}

uint8_t toShiftInput(bool shouldStepValues[NUMBER_OF_SPLIT_FLAPS])
{
  uint8_t shiftInput = 0;
  for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
    if (shouldStepValues[i]) {
      shiftInput |= 1 << (MAX_SPLIT_FLAPS - 1 - i);
    }
  }

  return shiftInput;
}

void setSplitFlapArrayTargets(SplitFlap splitFlapArray[], uint8_t displayCharacters[])
{
  // Set flap targets for each character in word
  for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
    uint8_t flapCharacter = displayCharacters[i];
    splitFlapArray[i].setFlapTarget(flapCharacter);
  }
}

bool hasSplitFlapArrayReachedTarget(SplitFlap splitFlapArray[])
{
  for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
    if (!splitFlapArray[i].isAtFlapTarget()) {
      return false;
    }
  }

  return true;
}

void shiftOutSteps(uint8_t shiftInput)
{
  // Prevent data from being released
  digitalWrite(SR_LATCH_PIN, LOW);
  // Write the step data to register
  shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, LSBFIRST, shiftInput);
  // Release the data
  digitalWrite(SR_LATCH_PIN, HIGH);
}

void stepSplitFlapArrayOnce(uint8_t shiftInput)
{
  shiftOutSteps(shiftInput);
  delayMicroseconds(PULSE_DELAY);
  shiftOutSteps(0);
  delayMicroseconds(PULSE_DELAY);
}

void stepSplitFlapArray(SplitFlap splitFlapArray[])
{
  bool splitFlapsToStep[NUMBER_OF_SPLIT_FLAPS];

  // Update the internal state of each split flap
  for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i)
  {
    bool isFlapAtTarget = splitFlapArray[i].step();

    splitFlapsToStep[i] = !isFlapAtTarget;
  }

  const uint8_t shiftInput = toShiftInput(splitFlapsToStep);

  stepSplitFlapArrayOnce(shiftInput);
}

void resetFlaps(SplitFlap splitFlapArray[])
{
  // Reset the flap targets for each splitFlap
  for (int i = 0; i < NUMBER_OF_SPLIT_FLAPS; ++i) {
    splitFlapArray[i].reset();
  }

  // TODO: Use shift register
  // For now just step all until single sensor reached
  while(splitFlapArray[0].isResetting()) {
    stepSplitFlapArrayOnce(B11111111);
  }
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

  int wordIndex = 0;
  char character = word[0];

  while (character != '\0') {
    character = word[wordIndex];
    if (character && wordIndex < NUMBER_OF_SPLIT_FLAPS) {
      splitFlapArray[wordIndex].setFlapTarget(character);
    }
    ++wordIndex;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(SR_LATCH_PIN, OUTPUT);
  pinMode(SR_CLOCK_PIN, OUTPUT);
  pinMode(SR_DATA_PIN, OUTPUT);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);

  digitalWrite(DIR_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), ISR_SENSOR, FALLING);

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


  // Ensure all split flaps start from blank
  // resetFlaps(splitFlapArray);
}

void loop() {
  // Maintain a connection to the server
  client.loop();

  if (!client.connected()) {
      Serial.println(F("Lost connection, reconnecting..."));
      connectAWS();
  }

  delay(1000);

  while (!hasSplitFlapArrayReachedTarget(splitFlapArray)) {
    stepSplitFlapArray(splitFlapArray);
  }

}