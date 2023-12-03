// Sensor.cpp
#include "Sensor.h"

Sensor::Sensor(int pin, char mode) {
  sensorPin = pin;
  sensorOutput = 42;
  sensorMode = mode;

  if (sensorMode == 'A'){
    adc.begin();
  }else if (sensorMode == 'D'){
    pinMode(sensorPin, INPUT);
  }
}

int Sensor::getSensorData() {
  if (sensorMode == 'A'){
    for (int i=0; i<10; i++){
      sensorOutput += adc.analogRead(sensorPin);
      delay(30)
    }

    float avgSensorOutput = sensorOutput/10;
    sensorOutput = static_cast<int>(avgSensorOutput/1024 * 3300); //Voltage in milliVolt
    
  }else if (sensorMode == 'D'){
    sensorOutput = digitalRead(sensorPin);
  }
  return sensorOutput;
}