// Software.ino

#include <MCP3XXX.h>
#include "Adafruit_SHT31.h"
#include "StandardSensor.h"


int TEMP_HUM_PIN = 0; //A0 on ESP

//Pins on ADC
const int PH_PIN = 0;
const int EC_PIN = 1;
const int WATERTEMP_PIN = 3;
const int WATERFLOW_PIN = 4;

//Digital Pins
const int WATERLEVEL_PIN = 0;

const int PUMP_REL_PIN = 1;
const int FERT_PUMP_PIN = 2;
const int PH_DOWN_PUMP_PIN = 3;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
StandardSensor waterlevelSensor(WATERLEVEL_PIN, 'D');

float air_temp;
float humidity;
float pH_value;
float ec_value;
int waterlevel;
float watertemp;
float waterflow;

void setup(){
  Serial.begin(9600);
  sht31.begin();
}

void loop(){
  
  measureData();

  Serial.println("..................................................");
  Serial.print("Waterlevel: "); Serial.println(waterlevel);
  delay(1000);
}

void getTempAndHumidity(int pin){
  air_temp = sht31.readTemperature();
  humidity = sht31.readHumidity();
  Serial.print("Temperature: "); Serial.print(air_temp); Serial.print(" C     Humidity: "); Serial.print(humidity); Serial.println("%");
  Serial.println();
}

void measureData(){
  //getTempAndHumidity(TEMP_HUM_PIN);
  waterlevel = waterlevelSensor.getSensorData();
}




