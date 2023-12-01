// StandardSensor.cpp
#include "StandardSensor.h"

StandardSensor::StandardSensor(int pin, char mode) {
  sensorPin = pin;
  sensorOutput = 42;
  sensorMode = mode;

  if sensorMode == 'A'{
    adc.begin();
  }else if sensorMode == 'D'{
    pinMode(sensorPin, INPUT);
  }
}

int StandardSensor::getSensorData() {
  if sensorMode == 'A'{
    sensorOutput = adc.analogRead(sensorPin);
    return sensorOutput;
  }else if sensorMode == 'D'{
    sensorOutput = digitalRead(sensorPin)
  }
  
  return sensorOutput;
}