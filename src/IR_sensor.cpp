#include <Romi32U4.h>
#include "IR_sensor.h"

void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

float IRsensor::PrintData(void)
{
    Serial.println(ReadData());
}

float IRsensor::ReadData(void)
{
  float A = .000783;
  float B = -.105;
  //assignment 1.1
  //read out and calibrate your IR sensor, to convert readouts to distance in [cm]
  float ADCIR = analogRead(pin_IR);
  float distance = (6762/(ADCIR - 9))-4 ;
  return distance;
}