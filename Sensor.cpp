// Sensor.cpp
#include "Sensor.h"
#include <iostream>

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
    sensorOutput = adc.analogRead(sensorPin);
    
  }else if (sensorMode == 'D'){
    sensorOutput = digitalRead(sensorPin);
  }
  return sensorOutput;
}