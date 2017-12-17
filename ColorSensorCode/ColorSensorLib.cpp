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

String returnColor(int minOfAll){
   long distBlue = calDisColorBlue(r, g, b);
  if (minOfAll > distBlue) {
    minOfAll = distBlue;
    minString = "blue";
  }
  long distRed = calDisColorRed(r, g, b);
  if (minOfAll > distRed) {
    minOfAll = distRed;
    minString = "red";
  }

  long distGreen = calDisColorGreen(r, g, b);
  if (minOfAll > distGreen) {
    minOfAll = distGreen;
    minString = "green";
  }
  long distCyan = calDisColorCyan(r, g, b);
  if (minOfAll > distCyan) {
    minOfAll = distCyan;
    minString = "cyan";
  }
  long distYellow = calDisColorYellow(r, g, b);
  if (minOfAll > distYellow) {
    minOfAll = distYellow;
    minString = "yellow";
  }
  long distMagenta = calDisColorMagenta(r, g, b);
  if (minOfAll > distMagenta) {
    minOfAll = distMagenta;
    minString = "magenta";
  }
  long distGray = calDisColorGray(r, g, b);
  if (minOfAll > distGray) {
    minOfAll = distGray;
    minString = "gray";
  }
  long distBlack = calDisColorBlack(r, g, b);
  if (minOfAll > distBlack) {
    minOfAll = distBlack;
    minString = "black";
  }

  long distWhite = calDisColorWhite(r, g, b);
  if (minOfAll > distWhite) {
    minOfAll = distWhite;
    minString = "white";
  }
  return minString
}

long calDisColorRed(int cR, int cG, int cB) {
  //cR - myred.red
  long d1 = (cR - myred[0]);
  d1 = d1 * d1;
  //cG - myred.green
  long d2 = (cG - myred[1]);
  d2 = d2 * d2;
  //cB - myred.blue
  long d3 = (cB - myred[2]);
  d3 = d3 * d3;
  return sqrt(d1 + d2 + d3);
}

long calDisColorBlue(int cR, int cG, int cB) {
  //cR - myblue.red
  long d1 = cR - myblue[0];
  d1 = d1 * d1;
  //cG - myblue.green
  long d2 = cG - myblue[1];
  d2 = d2 * d2;
  //cB - myblue.blue
  long d3 = cB - myblue[2];
  d3 = d3 * d3;
  return sqrt(d1 + d2 + d3);
}

long calDisColorGreen(int cR, int cG, int cB) {
  //cR - mygreen.red
  long d1 = cR - mygreen[0];
  d1 = d1 * d1;
  //cG - mygreen.green
  long d2 = cG - mygreen[1];
  d2 = d2 * d2;
  //cB - mygreen.blue
  long d3 = cB - mygreen[2];
  d3 = d3 * d3;
  return sqrt(d1 + d2 + d3);
}

long calDisColorYellow(int cR, int cG, int cB) {
  //cR - yellow.red
  long d1 = cR - yellow[0];
  d1 = d1 * d1;
  //cG - yellow.green
  long d2 = cG - yellow[1];
  d2 = d2 * d2;
  //cB - yellow.blue
  long d3 = cB - yellow[2];
  d3 = d3 * d3;
  return sqrt(d1 + d2 + d3);
}

long calDisColorCyan(int cR, int cG, int cB) {
  //cR - cyan.red
  long d1 = cR - cyan[0];
  d1 = d1 * d1;
  //cG - cyan.green
  long d2 = cG - cyan[1];
  d2 = d2 * d2;
  //cB - cyan.blue
  long d3 = cB - cyan[2];
  d3 = d3 * d3;
  return sqrt(d1 + d2 + d3);
}

long calDisColorMagenta(int cR, int cG, int cB) {
  //cR - magenta.red
  long d1 = cR - magenta[0];
  d1 = d1 * d1;
  //cG - myred.green
  long d2 = cG - magenta[1];
  d2 = d2 * d2;
  //cB - myred.blue
  long d3 = cB - magenta[2];
  d3 = d3 * d3;
  return sqrt(d1 + d2 + d3);
}

long calDisColorGray(int cR, int cG, int cB) {
  //cR - magenta.red
  long d1 = cR - gray[0];
  d1 = d1 * d1;
  //cG - myred.green
  long d2 = cG - gray[1];
  d2 = d2 * d2;
  //cB - myred.blue
  long d3 = cB - gray[2];
  d3 = d3 * d3;
  return sqrt(d1 + d2 + d3);
}

long calDisColorBlack(int cR, int cG, int cB) {
  //cR - black.red
  long d1 = cR - black[0];
  d1 = d1 * d1;
  //cG - myred.green
  long d2 = cG - black[1];
  d2 = d2 * d2;
  //cB - myred.blue
  long d3 = cB - black[2];
  d3 = d3 * d3;
  return sqrt(d1 + d2 + d3);
}

long calDisColorWhite(int cR, int cG, int cB) {
  //cR - black.red
  long d1 = cR - white[0];
  d1 = d1 * d1;
  //cG - myred.green
  long d2 = cG - white[1];
  d2 = d2 * d2;
  //cB - myred.blue
  long d3 = cB - white[2];
  d3 = d3 * d3;
  return sqrt(d1 + d2 + d3);
}


