// Software.ino

#include <tuple>
#include <iostream>
#include <MCP3XXX.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_EC10.h"
#include <EEPROM.h>
#include "DHT.h"
#include "Sensor.h"

#define TEMP_HUM_PIN A0 //A0 on ESP
#define TEMP_HUM_TYPE DHT22

//Pins on ADC
const int PH_PIN = 0;
const int EC_PIN = 1;

//Digital Pins
const int WATERLEVEL_PIN = 0;
const int WATERTEMP_PIN = 1;

const int PUMP_REL_PIN = 1;
const int FERT_PUMP_PIN = 2;
const int PH_DOWN_PUMP_PIN = 3;

DHT tempHumSensor(TEMP_HUM_PIN, TEMP_HUM_TYPE);
Sensor waterlevelSensor(WATERLEVEL_PIN, 'D');
Sensor pHSensor(PH_PIN, 'A');
DFRobot_EC10 ec;
Sensor ecSensor(EC_PIN, 'A');
OneWire oneWire(WATERTEMP_PIN);
DallasTemperature watertempSensor(&oneWire);

float air_temp;
float humidity;
float pH_value;
float ec_voltage;
float ec_value;
int waterlevel;
float watertemp = 20;

void setup(){
  Serial.begin(9600);
  tempHumSensor.begin();
  ec.begin();
  watertempSensor.begin();
}

void loop(){
  
  measureData();

  delay(2000);
}

void measureData(){
  static unsigned long timepoint = millis();
  if(millis()-timepoint>2000U){  //time interval: 2s
    timepoint = millis();
    
    /*
    air_temp = tempHumSensor.readTemperature();
    humidity = tempHumSensor.readHumidity();
    Serial.print("Temperature: "); Serial.print(air_temp); Serial.print(" C     Humidity: "); Serial.print(humidity); Serial.println("%"); Serial.println(); //Output Temp & Humidity
    */

    watertempSensor.requestTemperatures();
    watertemp = watertempSensor.getTempCByIndex(0);

    float pH_Voltage = pHSensor.getSensorData() / 1024 * 3.3; 
    
    ec_voltage = ecSensor.getSensorData()/1024.0*3300;   // read the voltage
    ec_value =  ec.readEC(ec_voltage, watertemp);  // convert voltage to EC with temperature compensation
    Serial.print("temperature:"); Serial.print(watertemp,1); Serial.print("^C  EC:"); Serial.print(ec_value,2); Serial.println("ms/cm");

    waterlevel = waterlevelSensor.getSensorData();
    Serial.print("Waterlevel: "); Serial.println(waterlevel); Serial.println();
  }
  ec.calibration(ec_voltage, watertemp);
}




