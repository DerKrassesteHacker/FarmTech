// StandardSensor.h

#ifndef StandardSensor_h
#define StandardSensor_h

#include <Arduino.h>
#include <MCP3XXX.h>

class StandardSensor {
  private:
    int sensorPin;
    int sensorOutput;
    char sensorMode;
    MCP3008 adc;

  public:
    StandardSensor(int pin, char mode);
    int getSensorData();
};

#endif
