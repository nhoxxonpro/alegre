#include "Arduino.h"
#include "ColorSensor.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"
// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
int myred[3] =     {195, 40, 45};
int mygreen[3] =   {79, 100, 80};
int myblue[3] =    {47, 80, 134};
int yellow[3] =    {130, 90, 40};
int cyan[3] =      {65, 105, 86};
int magenta[3] =   {125, 65, 80};
int gray[3] =      {85, 94, 83};
int black[3] =     {110, 87, 72};
int white[3] =     {91, 91, 75};
long minOfAll = 1000;

ColorSensorLib::ColorSensorLib(){
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
}

void ColorSensorLib::SETUP(){
  
}



