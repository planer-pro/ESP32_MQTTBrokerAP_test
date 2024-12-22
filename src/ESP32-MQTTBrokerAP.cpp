/*
Base code for an ESP32 MQTT broker avaiable via its own AP.
No RPi-Mosquitto broker, cloud broker or wifi/router needed.
    - Broker runs on core 0, local client and main code on core 1 (default).
    - tested on phone with MQTT Analyzer and on PC with MQTT Explorer (simultaneously).
    - tested with two ESP8266 (D1 Mini) clients connected to D1 Mini ESP32 version as host-broker.
*/
#include "Arduino.h"
#include <sMQTTBroker.h>  // https://github.com/terrorsl/sMQTTBroker
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient

#define LED (5)
#define mqttPort 1883
#define mqttServer "127.0.0.1"

sMQTTBroker broker;
WiFiClient espClient;
PubSubClient MQTTclient(espClient);

uint32_t _tm = 0, _tmLed = 0;
uint16_t interval = 1000, ledInterval = 50;
char testcount[3];
bool wifiModeAP = false, enLed = false;
uint8_t testCount;

TaskHandle_t brokerTask;

void ledBlinkHandler()
{
    if (enLed && (millis() - _tmLed) > ledInterval)
    {
        _tmLed = millis();

        if (digitalRead(LED))
        {
            digitalWrite(LED, LOW);

            _tmLed = millis();
        }
        else
        {
            digitalWrite(LED, HIGH);

            enLed = false;
        }
    }
}

void brokerTaskCode(void *pvParameters)
{
    Serial.print("broker running on core ");
    Serial.println(xPortGetCoreID());

    for (;;)
    {
        delay(1000);
        broker.update();
    }
}

void reconnect()
{
    /* Loop until (re)connected */
    while (!MQTTclient.connected())
    {
        Serial.print("Attempting MQTT connection...");

        /* Attempt to connect */
        if (MQTTclient.connect("ESP32"))
        {
            Serial.println("connected");
            /* ... and resubscribe */
            MQTTclient.subscribe("test");
            MQTTclient.subscribe("mobile");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(MQTTclient.state());
            Serial.println(" will try again in 5secs");
            delay(5000); // Wait 5 seconds before retrying
        }
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived:\ttopic: ");
    Serial.print(topic);

    String payloadStr = String((char *)payload).substring(0, length);

    if (String(topic) == "test") // matching topic?
        Serial.println("\tpayload: " + payloadStr);
    if (String(topic) == "mobile")
        Serial.println("\tpayload: " + payloadStr);

    enLed = true;
}

void setup()
{
    Serial.begin(115200);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    /*  create task for broker that will be executed on core 0  */
    xTaskCreatePinnedToCore(
        brokerTaskCode, /* Task function. */
        "brokerTask",   /* name of task. */
        10000,          /* Stack size of task */
        NULL,           /* parameter of the task */
        1,              /* priority of the task */
        &brokerTask,    /* Task handle to keep track of created task */
        0);             /* pin task to core 0 */

    delay(500);

    const char *ssid = "ESP32";            // SSID
    const char *password = "kolos025_2.4"; // password

    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();

    Serial.print("AP IP address: ");
    Serial.println(myIP);

    broker.init(mqttPort);
    delay(1500);

    MQTTclient.setServer(mqttServer, mqttPort);
    MQTTclient.setCallback(callback);
}

void loop()
{
    if (!MQTTclient.connected())
        reconnect();

    MQTTclient.loop();

    ledBlinkHandler();

    /*For testing publish count every second*/
    if ((millis() - _tm) >= interval) // time between updates
    {
        _tm = millis();

        /* example counter */
        itoa(testCount++, testcount, 10);
        broker.publish("test", testcount);
    }
}