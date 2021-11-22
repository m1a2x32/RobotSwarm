/*
  SimpleMQTTClient.ino
  The purpose of this exemple is to illustrate a simple handling of MQTT and Wifi connection.
  Once it connects successfully to a Wifi network and a MQTT broker, it subscribe to a topic and send a message to it.
  It will also send a message delayed 5 seconds later.
*/

#include "EspMQTTClient.h"
#include "ArduinoJson.h"

#define BUF_LEN 256
#define SENDDISTANCE "$SendDistance"
String message, distance, lrpm, rrpm;
bool automatic = true;
unsigned long prevMillis;


EspMQTTClient client(
  "AndroidAP8347",
  "12345678",
  "test.mosquitto.org", // MQTT Broker
  "i425447",
  "12345",
  "TomaESP",     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);

String Split(String data, char separator , int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

String GetDistance()
{
  if (Serial2.available() > 0)
  {
    
    message = Serial2.readStringUntil('#');
    distance = Split(message, ':', 1);
    return distance;
  }
  return "Disconnected";
}

String GetLeftRpm()
{
  if (Serial2.available() > 0)
  {

    message = Serial2.readStringUntil('#');
    lrpm = Split(message, ':', 2);
    return lrpm;
  }
  return "Disconnected";
}

String GetRightRpm()
{
  if (Serial2.available() > 0)
  {

    message = Serial2.readStringUntil('#');
    rrpm = Split(message, ':', 3);
    return rrpm;
  }
  return "Disconnected";
}



void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);

  // Optionnal functionnalities of EspMQTTClient :
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overrited with enableHTTPWebUpdater("user", "password").
  client.enableLastWillMessage("TestClient/lastwill", "I am going offline");  // You can activate the retain flag by setting the third parameter to true
  // SetUp of DHT22 Sensor


}

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
  // Subscribe to "nr_workshop/test" and display received message to Serial
  client.subscribe("robot_swarm/distance", [](const String & payload) {

    if (payload == "Manual")
    {
      automatic = false;
      Serial2.print('M');
    }
    else if (payload == "Automatic")
    {
      automatic = true;
      Serial2.print('A');
    }

    if (automatic == false)
    {
      if (payload == "Forward")
        Serial2.print('F');
      else if (payload == "Left")
        Serial2.print('L');
      else if (payload == "Right")
        Serial2.print('R');
      else if (payload == "Stop")
        Serial2.print('S');
    }
  });
}

void loop()
{
  //if (millis() - prevMillis > 100)
  //{
    char buffer[BUF_LEN];
    StaticJsonDocument<BUF_LEN> doc;
    client.loop();
    //delay(1);
    doc["distance"] = GetDistance();
    doc["lrpm"] = GetLeftRpm();
    doc["rrpm"] = GetRightRpm();

    serializeJson(doc, buffer);
    client.publish("robot_swarm/distance", buffer);
   // prevMillis = millis();
  //}
  client.loop();
}
