#include <Romi32U4.h>
#include "IR_sensor.h"
void IRsensor::Init(int pin)
{
    pinMode(pin, INPUT);
    pin_IR = pin;
}

float IRsensor::PrintData(void)
{
    Serial.println(ReadData(), 5);
}

float IRsensor::ReadData(void)
{
  //assignment 1.1
  //read out and calibrate your IR sensor, to convert readouts to distance in [cm]
  //Through calibration found that the equation for voltage (y) in terms of 1/distance (x) is y = 16.83x + 0.54
  // So x = (y - 0.54) / 16.83
  int IRDistanceADC = analogRead(pin_IR);
  float IRDistVoltage = ((float)IRDistanceADC / 1023.0) * 5.0;
  float distance = 1 / ((IRDistVoltage - 0.54) / 16.83);
  return distance;

}