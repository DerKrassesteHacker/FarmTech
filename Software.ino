// Software.ino

//TODO: Delays einbauen
//TODO: Licht-Sensor
//TODO: Einzelne Daten an Server senden
//TODO: Daten über 10min averagen und in Liste auf Server ablegen
//TODO: Alle Sensoren ausprobieren; Passen die Delays?
//TODO: Dosierpumpen ansteuern

#include <tuple>
#include <iostream>
#include <vector>
using std::vector;

#include <MCP3XXX.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_EC10.h"
#include <EEPROM.h>
#include "DHT.h"
#include "Sensor.h"
#include <Redis.h>

#define TEMP_HUM_TYPE DHT22

//Pins on ADC
const int PH_PIN = 0;
float pH_gradient = -6.667;
float pH_calibration_value = 24.47; // 21.34;
const int EC_PIN = 1;
const int LIGHT_PIN = 2;

//Digital Pins
const int TEMP_HUM_PIN = 0; //GPIO0 on ESP
const int WATERLEVEL_PIN = 1;
const int WATER_TEMP_PIN = 2;

//Output Pins
const int FERT_PUMP_PIN = 3;
const int PH_DOWN_PUMP_PIN = 4;

DHT tempHumSensor(TEMP_HUM_PIN, TEMP_HUM_TYPE);
Sensor waterlevelSensor(WATERLEVEL_PIN, 'D');
Sensor pHSensor(PH_PIN, 'A');
DFRobot_EC10 ec;
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
float ph_value;
float ec_voltage;
float ec_value;
float light_value;

int waterlevel;

long tempHumTime = millis();
long waterlevelTime = millis();
long watertempTime = millis();
long ecTime = millis();
long phTime = millis();
long lightTime = millis();

long averagingTime = millis();


#define WIFI_SSID       "Leberwurst"
#define WIFI_PASSWORD   "lustverkantung43beta"

#define REDIS_ADDR      "2001:16b8:317f:6400:308b:afe9:7b96:d378" 
#define REDIS_PORT      6379 
#define REDIS_PASSWORD  "Nudelsuppe"  

Redis redis(REDIS_ADDR, REDIS_PORT);


void setup(){
  Serial.begin(9600);
  tempHumSensor.begin();
  ec.begin();
  water_tempSensor.begin();

  startRedis();
}

void loop(){

  measureData();

  if(millis()-averagingTime > 2000){  //time interval: 2s
    averageData();
    //Check Relais
  }

  //Save Statistical Data

}

void measureData(){
  if(millis()-tempHumTime > 500){  //time interval: 0.5s
    float temp = tempHumSensor.readTemperature();
    float hum = tempHumSensor.readHumidity();
    air_temp_vals.push_back(temp);
    humidity_vals.push_back(hum);

    Serial.print("Temperature: "); Serial.print(temp); Serial.print(" C     Humidity: "); Serial.print(hum); Serial.println("%"); Serial.println(); //Output Temp & Humidity

    tempHumTime = millis();
  }

  if(millis() - watertempTime > 750){  //time interval: 0.75s
    water_tempSensor.requestTemperatures();
    float temp = water_tempSensor.getTempCByIndex(0);
    watertemp_vals.push_back(temp);

    watertempTime = millis();
  }
  
  if(millis() - phTime > 30){  //time interval: 30ms
    int ph_analog_val = pHSensor.getSensorData(); //conversion from mV to V
    float pH_voltage = float(ph_analog_val)/1024 * 3.3;
    float ph_value = pH_gradient * pH_voltage + pH_calibration_value;
    ph_values.push_back(ph_value);
    Serial.print("pH-Wert: "); Serial.print(ph_value); Serial.println();

    phTime = millis();
  }

  if(millis() - ecTime > 30){  //time interval: 30ms
    int ec_analog_val = ecSensor.getSensorData();   // read the voltage in mV
    float ec_voltage = float(ec_analog_val)/1024 * 3.3;
    float ec_value = ec.readEC(ec_voltage, watertemp);  // convert voltage to EC with temperature compensation
    ec_values.push_back(ec_value);
    Serial.print("temperature:"); Serial.print(watertemp,1); Serial.print("^C  EC:"); Serial.print(ec_value,2); Serial.println("ms/cm");

    ecTime = millis();
  }
  ec.calibration(ec_voltage, watertemp);

  if(millis() - lightTime > 30){  //time interval: 30ms
    int light_analog_val = lightSensor.getSensorData();
    float light_voltage = float(light_analog_val)/1024 * 3.3;
    //float light_value = light_gradient * light_voltage + light_calibration_val;  // convert voltage to EC with temperature compensation
    light_values.push_back(light_voltage);
    Serial.print("Licht Widerstand:"); Serial.println(light_voltage);

    lightTime = millis();
  }

  if(millis() - waterlevelTime > 500){  //time interval: 0.5s
    waterlevel = waterlevelSensor.getSensorData();
    Serial.print("Waterlevel: "); Serial.println(waterlevel); Serial.println();

    waterlevelTime = millis();
  }

}

void startRedis(){
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to the WiFi");
  while (WiFi.status() != WL_CONNECTED){
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

void averageData(){
  float air_temp = avgValue(air_temp_vals);
  SaveAndSend(air_temp);
  float humidity = avgValue(humidity_vals);
  SaveAndSend(humidity);
  float watertemp = avgValue(watertemp_vals);
  SaveAndSend(watertemp);
  float ph_value = avgValue(ph_values);
  SaveAndSend(ph_value);
  float ec_value = avgValue(ec_values);
  SaveAndSend(ec_value);
  float light_value = avgValue(light_values);
  SaveAndSend(light_value);
}

float avgValue(vector<float> values){
  float sum = 0.00;
  int length = end(values) - begin(values);
    for(int i=0; i<length; i++){
        sum+=values[i];
    }
  float avg = sum / length;
  return avg;
}

void SaveAndSend(float value){

}
