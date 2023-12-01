#include <Redis.h>
#include "DHT.h"

#define DHTPIN 12 // Signal pin on DHT11
#define DHTTYPE DHT11   // DHT 11 is used
DHT dht(DHTPIN, DHTTYPE);

const int flameAiPin = 13;
const int flameDiPin = 14;

#define WIFI_SSID       "FRIED"
#define WIFI_PASSWORD   "diehlamfriedhof"

#define REDIS_ADDR      "192.168.0.121" 
#define REDIS_PORT      6379 
#define REDIS_PASSWORD  ""  

Redis redis(REDIS_ADDR, REDIS_PORT);

void setup()
{
    Serial.begin(9600);
    Serial.println();

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to the WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(250);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    if (redis.begin(REDIS_PASSWORD))
    {
        Serial.println("Connected to the Redis server!");
    }
    else
    {
        Serial.println("Failed to connect to the Redis server!");
        return;
    }
}

void loop()
{
  char themsg[10];
  // Get and publish the room temperature
  sprintf(themsg,"%d",int(dht.readTemperature()));
  redis.publish("roomtemp", themsg);

  // Get and publish the flame intensity
  sprintf(themsg,"%d",analogRead(flameAiPin));
  redis.publish("flameintensity", themsg); 

  // Get and publish the flame status
  if (digitalRead(flameDiPin) == 1) {
    redis.publish("flamestatus", "FIRE ON");
  } else {
    redis.publish("flamestatus", "NO FLAME");
  }
  delay(5000);

}