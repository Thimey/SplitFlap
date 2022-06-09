
#include <WiFiClientSecure.h>
#include <MQTTClient.h>

#include "config.h"
#include "secrets.h"
#include "Comms/Comms.h";


// WiFi client
WiFiClientSecure net = WiFiClientSecure();
// MQTT client
MQTTClient client = MQTTClient(256);

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
    client.subscribe(START_TIMER_TOPIC);

    Serial.println("AWS IoT Connected!");
}

void commsLoop() {
    client.loop();

    if (!client.connected()) {
        Serial.println(F("Lost connection, reconnecting..."));
        connectAWS();
    }
}


void commsInitialise(MessageHandler messageHandler)
{
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


