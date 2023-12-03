// Sensor.h

#ifndef Sensor_h
#define Sensor_h

#include <Arduino.h>
#include <MCP3XXX.h>

class Sensor {
  private:
    int sensorPin;
    int sensorOutput;
    float voltage;
    char sensorMode;
    MCP3008 adc;

  public:
    Sensor(int pin, char mode);
    int getSensorData();
};

#endif
