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
  //read out IR and return dist
  int IRDistanceADC = analogRead(pin_IR);
  float IRDistVoltage = ((float)IRDistanceADC / 1023.0) * 5.0;
  float distance = 1 / ((IRDistVoltage - 0.54) / 16.83);
  return distance;

}