// 
// From: Redis IoT Blog sample
// 2017 F.Cerbelle <francois@cerbelle.net>
//
// Source code related to the blog post
// https://fcerbell.github.io/en/2017-09-12-IotSensorSendingToRedis/
//

//  your network SSID (name)
#define WIFI_SSID "Leberwurst"
#define WIFI_PASS "lustverkantung43beta"

// Redis details
#define REDISHOST "127.0.0.1"
#define REDISPORT 6379

#include <ESP8266WiFi.h>

unsigned long lastSensorRead=0;
WiFiClient redis;
unsigned long lastValue=0;
char szValue[6]="";


void setup() {
  // Serial console initialization for debugging
  Serial.begin(115200);
//  Serial.setDebugOutput(true); // Wifi debugging
  while (!Serial);
  Serial.println("Serial initialized.");

  // WIFI connection
//  ESP.eraseConfig();           // in case of need
//  ESP.reset();                 // in case of need
//  WiFi.softAPdisconnect(true); // in case of need
//  WiFi.disconnect(true);       // in case of need
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  // Blink LED twice per second while waiting for connection
  pinMode(LED_BUILTIN, OUTPUT);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi (");
  Serial.print(WiFi.macAddress());
  Serial.print(") connected with IP ");
  Serial.println(WiFi.localIP());
  Serial.print("DNS0 : ");
  Serial.println(WiFi.dnsIP(0));
  Serial.print("DNS1 : ");
  Serial.println(WiFi.dnsIP(1));
}

void loop() {
  if ((millis() - lastSensorRead)>5000) {
    lastSensorRead = millis();
    lastValue = analogRead(0);
    Serial.print("Sensor value (0-1024) : ");
    Serial.println(lastValue);
    if (!redis.connected()) {
      Serial.print("Redis not connected, connecting...");
      if (!redis.connect(REDISHOST,REDISPORT)) {
        Serial.print  ("Redis connection failed...");
        Serial.println("Waiting for next read");
        return; 
      } else
        Serial.println("OK");
    }

    // 10 is the base
    ltoa(lastValue,szValue,10);
    redis.print(
      String("*3\r\n")
      +"$5\r\n"+"LPUSH\r\n"
      +"$19\r\n"+"v:"+WiFi.macAddress().c_str()+"\r\n"
      +"$"+strlen(szValue)+"\r\n"+szValue+"\r\n"
    );
  }
  // If there is an answer from the Redis server available
  // Consume Redis server replies from the buffer and discard them
  while (redis.available() != 0)
    Serial.print((char)redis.read());
}
