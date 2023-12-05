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

#define TEMP_HUM_TYPE DHT22

//Pins on ADC
const int PH_PIN = 0;
float pH_calibration_value = 0.0; // 21.34;
float pH_gradient = 1.0;
const int EC_PIN = 1;
const int LIGHT_PIN = 2;


//Digital Pins
const int TEMP_HUM_PIN 0 //GPIO0 on ESP
const int WATERLEVEL_PIN = 1;
const int WATERTEMP_PIN = 2;

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


}

void measureData(){
  static unsigned long timepoint = millis();
  if(millis()-timepoint>2000U){  //time interval: 2s
    timepoint = millis();


    air_temp = tempHumSensor.readTemperature();
    humidity = tempHumSensor.readHumidity();
    Serial.print("Temperature: "); Serial.print(air_temp); Serial.print(" C     Humidity: "); Serial.print(humidity); Serial.println("%"); Serial.println(); //Output Temp & Humidity
    delay(500);

    watertempSensor.requestTemperatures();
    watertemp = watertempSensor.getTempCByIndex(0);

    float pH_voltage = pHSensor.getSensorData() / 1000; //conversion from mV to V
    pH_value = pH_gradient * pH_voltage + pH_calibration_value;

    ec_voltage = ecSensor.getSensorData();   // read the voltage in mV
    ec_value =  ec.readEC(ec_voltage, watertemp);  // convert voltage to EC with temperature compensation
    Serial.print("temperature:"); Serial.print(watertemp,1); Serial.print("^C  EC:"); Serial.print(ec_value,2); Serial.println("ms/cm");

    waterlevel = waterlevelSensor.getSensorData();
    Serial.print("Waterlevel: "); Serial.println(waterlevel); Serial.println();
    delay(500);
  }
  //ec.calibration(ec_voltage, watertemp);
}


