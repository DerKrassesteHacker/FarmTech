// Software.ino

#include <tuple>
#include <iostream>
#include <vector>
using std::vector;
#include <string>
using std::string;
using std::to_string;

#include <MCP3XXX.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_ESP_EC.h"
#include <EEPROM.h>
#include "DHT.h"
#include "Sensor.h"
#include <Redis.h>
#include <ESP8266WiFi.h>
#include <ESPDateTime.h>

#define TEMP_HUM_TYPE DHT22

//Pins on ADC
const int PH_PIN = 0;
const int EC_PIN = 1;
const int LIGHT_PIN = 2;

//Digital Pins
const int TEMP_HUM_PIN = 0;  //GPIO0 on ESP
const int WATERLEVEL_PIN = 1;
const int WATER_TEMP_PIN = 2;

//Output Pins
const int FERT_PUMP_PIN = 3;
const int PH_DOWN_PUMP_PIN = 4;
const int EC_VCC_PIN = 5;
const int PH_VCC_PIN = 16;

String activated_pin = "ph";

DHT tempHumSensor(TEMP_HUM_PIN, TEMP_HUM_TYPE);
Sensor waterlevelSensor(WATERLEVEL_PIN, 'D');
Sensor pHSensor(PH_PIN, 'A');
DFRobot_ESP_EC ec;
Sensor ecSensor(EC_PIN, 'A');
Sensor lightSensor(LIGHT_PIN, 'A');
OneWire oneWire(WATER_TEMP_PIN);
DallasTemperature water_tempSensor(&oneWire);

vector<float> air_temp_vals;
vector<float> humidity_vals;
vector<float> watertemp_vals;
vector<float> ph_values;
vector<float> ec_values;
vector<float> light_values;

float air_temp;
float humidity;
float watertemp = 20;
float pH_voltage;
float ph_value;
float ec_voltage;
float ec_value;
float light_value;

bool waterlevel;

float pH_gradient = -6.667;         //-7.143; //-6.667
float pH_calibration_value = 24.0;  //7.00; // 21.34;
float light_gradient = 1.0;
float light_calibration_value = 0.0;
int fert_dosing_quantity = 5;
int ph_dosing_quantity = 5;
float minimal_ec_value = 0.8;

vector<float> air_temp_stats;
vector<float> humidity_stats;
vector<float> watertemp_stats;
vector<float> ph_stats;
vector<float> ec_stats;
vector<float> light_stats;

long tempHumTime = millis();
long waterlevelTime = millis();
long watertempTime = millis();
long ecTime = millis();
long phTime = millis();
long ecAndpHTime = millis();
long lightTime = millis();

long averagingTime = millis();

long statisticTime = millis();

long pump_intervall = millis();
long pump_time;

bool ph_pump_status = false;
bool fert_pump_status = false;


#define WIFI_SSID "foo"
#define WIFI_PASSWORD "foobarbaz"

#define REDIS_ADDR "hydroponik.ddns.net"
#define REDIS_PORT 6379
#define REDIS_PASSWORD "foobar"

#define MAX_BACKOFF 300000  // 5 minutes

WiFiClient WifiClient;
Redis redis(WifiClient);


void setup() {
  Serial.begin(9600);
  tempHumSensor.begin();
  ec.begin();
  water_tempSensor.begin();

  pinMode(EC_VCC_PIN, OUTPUT);
  digitalWrite(EC_VCC_PIN, LOW);
  pinMode(PH_VCC_PIN, OUTPUT);
  digitalWrite(PH_VCC_PIN, HIGH);

  pinMode(PH_DOWN_PUMP_PIN, OUTPUT);
  pinMode(FERT_PUMP_PIN, OUTPUT);

  connectToWifi();

  DateTime.setTimeZone("CET-1CEST,M3.5.0,M10.5.0/3");
  DateTime.begin();
  //Source: https://github.com/mcxiaoke/ESPDateTime

  if (!DateTime.isTimeValid()) {
    Serial.println("Failed to get time from server.");
  }

  connectToServer();
}

void loop() {

  if (!WifiClient.connected()) {
    connectToServer();
  }

  appControlling();

  measureData();

  if (millis() - averagingTime > 3000) {                                                    //Every 3 Seconds
    String arduinoConnectionStr = DateTime.format("\"Last connected %a, %d.%m.%G, %X.\"");  //Source: https://cplusplus.com/reference/ctime/strftime/
    char arduinoConnection[arduinoConnectionStr.length() + 1];
    arduinoConnectionStr.toCharArray(arduinoConnection, arduinoConnectionStr.length() + 1);
    redis.set("Hydroponik:ArduinoConnection", arduinoConnection);

    averageData();
    averagingTime = millis();
  }


  if (millis() - pump_intervall > 300000) {  //time interval: 5min
    if (avgValue(ec_stats) <= minimal_ec_value) {
      pump_time = millis();
      fert_pump_status = true;
      digitalWrite(FERT_PUMP_PIN, HIGH);
    } else if (avgValue(ph_stats) >= 6.30) {
      pump_time = millis();
      ph_pump_status = true;
      digitalWrite(PH_DOWN_PUMP_PIN, HIGH);
    }
    pump_intervall = millis();
  }

  if (millis() - pump_time > fert_dosing_quantity * 1000 && fert_pump_status == true) {
    digitalWrite(FERT_PUMP_PIN, LOW);
    fert_pump_status = false;
    redis.set("Hydroponik:activateFertilizerPump", "false");
  } else if (millis() - pump_time > ph_dosing_quantity * 1000 && ph_pump_status == true) {
    digitalWrite(PH_DOWN_PUMP_PIN, LOW);
    ph_pump_status = false;
    redis.set("Hydroponik:activatePHDownPump", "false");
  }


  if (millis() - statisticTime > 600000) {  //time interval: 10min
    statisticalData();

    statisticTime = millis();
  }
}

void appControlling() {
  pH_gradient = redis.get("Hydroponik:ph_gradient").toFloat();
  pH_calibration_value = redis.get("Hydroponik:ph_calibration_value").toFloat();

  light_gradient = redis.get("Hydroponik:light_gradient").toFloat();
  light_calibration_value = redis.get("Hydroponik:light_calibration_value").toFloat();

  fert_dosing_quantity = redis.get("Hydroponik:fert_dosing_quantity").toInt();
  ph_dosing_quantity = redis.get("Hydroponik:ph_dosing_quantity").toInt();

  minimal_ec_value = redis.get("Hydroponik:minimal_ec_value").toFloat();

  if (redis.get("Hydroponik:activateFertilizerPump") == "true" && !fert_pump_status) {
    Serial.println("Fertilizer activated");
    pump_time = millis();
    fert_pump_status = true;
    digitalWrite(FERT_PUMP_PIN, HIGH);
  } else if (redis.get("Hydroponik:activatePHDownPump") == "true" && !ph_pump_status) {
    pump_time = millis();
    ph_pump_status = true;
    digitalWrite(PH_DOWN_PUMP_PIN, HIGH);
  }


  if (millis() - pump_time > 2000 && fert_pump_status == true) {
    digitalWrite(FERT_PUMP_PIN, LOW);
    fert_pump_status = false;
    redis.set("Hydroponik:activateFertilizerPump", "false");
  } else if (millis() - pump_time > 2000 && ph_pump_status == true) {
    digitalWrite(PH_DOWN_PUMP_PIN, LOW);
    ph_pump_status = false;
    redis.set("Hydroponik:activatePHDownPump", "false");
  }
}

void measureData() {
  if (millis() - tempHumTime > 500) {  //time interval: 0.5s
    float temp = tempHumSensor.readTemperature();
    float hum = tempHumSensor.readHumidity();
    air_temp_vals.push_back(temp);
    humidity_vals.push_back(hum);

    tempHumTime = millis();
  }

  if (millis() - watertempTime > 750) {  //time interval: 0.75s
    water_tempSensor.requestTemperatures();
    float water_temp = water_tempSensor.getTempCByIndex(0);
    watertemp_vals.push_back(water_temp);

    watertempTime = millis();
  }

  Serial.println(millis() - ecAndpHTime);
  Serial.println(activated_pin);

  if (millis() - ecAndpHTime < 2500) {  //EC- and pH-Sensor taking turn measuring data to avoid interference
    if (activated_pin != "ph") {
      digitalWrite(EC_VCC_PIN, LOW);
      delay(10);
      digitalWrite(PH_VCC_PIN, HIGH);
      activated_pin = "ph";
    }

    if (millis() - phTime > 30 && (millis() - ecAndpHTime) > 500) {  //time interval: 30ms
      int ph_analog_val = pHSensor.getSensorData();                  //conversion from mV to V
      pH_voltage = float(ph_analog_val) / 1024 * 3.3;
      float ph_value = pH_gradient * pH_voltage + pH_calibration_value;
      ph_values.push_back(ph_value);
      Serial.println(ph_value);

      phTime = millis();
    }
  } else if (millis() - ecAndpHTime < 5000) {
    if (activated_pin != "ec") {
      digitalWrite(PH_VCC_PIN, LOW);
      delay(10);
      digitalWrite(EC_VCC_PIN, HIGH);  // Activate operating voltage for EC-Sensor
      activated_pin = "ec";
    }

    if (millis() - ecTime > 30 && (millis() - ecAndpHTime) > 3000) {  //time interval: 30ms
      int ec_analog_val = ecSensor.getSensorData();                   // read bit value for voltage
      float ec_voltage = float(ec_analog_val) / 1024.0 * 3300;
      watertemp = 16;
      float ec_value = ec.readEC(ec_voltage, 16);  // convert voltage to EC with temperature compensation
      ec_values.push_back(ec_value);
      Serial.print("EC-Wert: ");
      Serial.println(ec_value);

      ec.calibration(ec_voltage, watertemp);

      ecTime = millis();
    }
  } else {
    ecAndpHTime = millis();
  }

  if (millis() - lightTime > 30) {  //time interval: 30ms
    int light_analog_val = lightSensor.getSensorData();
    float light_voltage = float(light_analog_val) / 1024 * 3.3;
    float light_value = light_gradient * light_voltage + light_calibration_value;  // convert voltage to value
    light_values.push_back(light_value);

    lightTime = millis();
  }

  if (millis() - waterlevelTime > 500) {  //time interval: 0.5s
    int waterlevelValue = waterlevelSensor.getSensorData();

    if (waterlevelValue == 0) {
      waterlevel = true;
    } else if (waterlevelValue == 1) {
      waterlevel = false;
    }

    waterlevelTime = millis();
  }
}

void connectToWifi() {
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to the WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void connectToServer() {
  //Source: https://github.com/electric-sheep-co/arduino-redis/blob/master/examples/Subscribe/Subscribe.ino

  if (!WifiClient.connect(REDIS_ADDR, REDIS_PORT)) {
    Serial.println("Failed to connect to the Redis server!");
    return;
  }

  Redis redis(WifiClient);
  auto connRet = redis.authenticate(REDIS_PASSWORD);
  if (connRet == RedisSuccess) {
    Serial.println("Connected to the Redis server!");
  } else {
    Serial.printf("Failed to authenticate to the Redis server! Errno: %d\n", (int)connRet);
    return;
  }
}

void averageData() {
  Serial.println("-------- Five Seconds Averages ---------");

  air_temp = avgValue(air_temp_vals);
  air_temp_vals.clear();
  addToStatisticalData(air_temp_stats, air_temp);

  char air_tempChar[8];
  dtostrf(air_temp, 6, 2, air_tempChar);  //Conversion to Datatype "char" for sending to the server
  redis.set("Hydroponik:air_temperature", air_tempChar);

  Serial.print("Temperature: ");
  Serial.print(air_temp);
  Serial.println(" C");  //Output to the console


  humidity = avgValue(humidity_vals);
  humidity_vals.clear();
  addToStatisticalData(humidity_stats, humidity);

  char humidityChar[8];
  dtostrf(humidity, 6, 2, humidityChar);
  redis.set("Hydroponik:humidity", humidityChar);

  Serial.print("Luftfeuchtigkeit: ");
  Serial.print(humidity);
  Serial.println("%");


  watertemp = avgValue(watertemp_vals);
  watertemp_vals.clear();
  addToStatisticalData(watertemp_stats, watertemp);

  char watertempChar[8];
  dtostrf(watertemp, 6, 2, watertempChar);
  redis.set("Hydroponik:water_temperature", watertempChar);

  Serial.print("Wassertemperatur: ");
  Serial.print(watertemp);
  Serial.println(" C");


  ph_value = avgValue(ph_values);
  ph_values.clear();
  addToStatisticalData(ph_stats, ph_value);

  char ph_valueChar[8];
  dtostrf(ph_value, 6, 2, ph_valueChar);
  redis.set("Hydroponik:ph_value", ph_valueChar);

  char pH_voltageChar[8];
  dtostrf(pH_voltage, 6, 2, pH_voltageChar);
  redis.set("Hydroponik:ph_voltage", pH_voltageChar);

  Serial.print("pH-Wert: ");
  Serial.println(ph_value);


  ec_value = avgValue(ec_values);
  ec_values.clear();
  addToStatisticalData(ec_stats, ec_value);

  char ec_valueChar[8];
  dtostrf(ec_value, 6, 2, ec_valueChar);
  redis.set("Hydroponik:ec_value", ec_valueChar);

  Serial.print("EC-Wert: ");
  Serial.print(ec_value);
  Serial.println(" ms/cm");


  light_value = avgValue(light_values);
  light_values.clear();
  addToStatisticalData(light_stats, light_value);

  char light_valueChar[8];
  dtostrf(light_value, 6, 2, light_valueChar);
  redis.set("Hydroponik:light_value", light_valueChar);

  Serial.print("Licht-Wert: ");
  Serial.println(light_value);

  if (waterlevel) {
    redis.set("Hydroponik:waterlevel", "true");
  } else {
    redis.set("Hydroponik:waterlevel", "false");
  }

  Serial.print("Wasser über Wasserstandssensor? ");
  Serial.println(waterlevel);

  Serial.println();
  Serial.println();
}

void statisticalData() {
  Serial.println("-------- 10 Minute Averages ---------");
  time_t timeRaw = DateTime.now();
  String time = (String)(timeRaw);

  Serial.print("Temperatur: ");
  Serial.print(avgValue(air_temp_stats));
  Serial.println(" C");
  //Formatting Data for Server Writing
  String air_temp_formatted_str = redis.get("Hydroponik:air_temp_stats") + time + "." + avgValue(air_temp_stats) + "$";
  char air_temp_formatted[air_temp_formatted_str.length() + 1];
  air_temp_formatted_str.toCharArray(air_temp_formatted, air_temp_formatted_str.length() + 1);
  redis.set("Hydroponik:air_temp_stats", air_temp_formatted);  //Writing Statistical Data to Server

  Serial.print("Luftfeuchtigkeit: ");
  Serial.print(avgValue(humidity_stats));
  Serial.println(" %");
  String humidity_formatted_str = redis.get("Hydroponik:humidity_stats") + time + "." + avgValue(humidity_stats) + "$";
  char humidity_formatted[humidity_formatted_str.length() + 1];
  humidity_formatted_str.toCharArray(humidity_formatted, humidity_formatted_str.length() + 1);
  redis.set("Hydroponik:humidity_stats", humidity_formatted);

  Serial.print("Wassertemperatur: ");
  Serial.print(avgValue(watertemp_stats));
  Serial.println(" C");
  String water_temp_formatted_str = redis.get("Hydroponik:water_temp_stats") + time + "." + avgValue(watertemp_stats) + "$";
  char water_temp_formatted[water_temp_formatted_str.length() + 1];
  water_temp_formatted_str.toCharArray(water_temp_formatted, water_temp_formatted_str.length() + 1);
  redis.set("Hydroponik:water_temp_stats", water_temp_formatted);

  Serial.print("ph-Wert: ");
  Serial.println(avgValue(ph_stats));
  String ph_value_formatted_str = redis.get("Hydroponik:ph_value_stats") + time + "." + avgValue(ph_stats) + "$";
  char ph_value_formatted[ph_value_formatted_str.length() + 1];
  ph_value_formatted_str.toCharArray(ph_value_formatted, ph_value_formatted_str.length() + 1);
  redis.set("Hydroponik:ph_value_stats", ph_value_formatted);

  Serial.print("EC-Wert: ");
  Serial.print(avgValue(ec_stats));
  Serial.println(" ms/cm");
  String ec_value_formatted_str = redis.get("Hydroponik:ec_value_stats") + time + "." + avgValue(ec_stats) + "$";
  char ec_value_formatted[ec_value_formatted_str.length() + 1];
  ec_value_formatted_str.toCharArray(ec_value_formatted, ec_value_formatted_str.length() + 1);
  redis.set("Hydroponik:ec_value_stats", ec_value_formatted);

  Serial.print("Licht-Wert: ");
  Serial.println(avgValue(light_stats));
  String light_value_formatted_str = redis.get("Hydroponik:light_value_stats") + time + "." + avgValue(light_stats) + "$";
  char light_value_formatted[light_value_formatted_str.length() + 1];
  light_value_formatted_str.toCharArray(light_value_formatted, light_value_formatted_str.length() + 1);
  redis.set("Hydroponik:light_value_stats", light_value_formatted);

  Serial.println();
  Serial.println();
}

float avgValue(vector<float> values) {
  float sum = 0.00;
  int length = end(values) - begin(values);
  for (int i = 0; i < length; i++) {
    sum += values[i];
  }
  float avg = sum / length;

  return avg;
}

void addToStatisticalData(vector<float> &dataVector, float value){
  dataVector.push_back(value);
  
  if (dataVector.size() > 200){
    dataVector.erase(dataVector.begin());
  }
}
