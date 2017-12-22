#include "Arduino.h"
#include "ColorSensorLib.h"
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
float r, g, b;
String minString;
uint16_t clear, red, green, blue;
//long calDisColorBlue;
//long calDisColorRed;
//long calDisColorGreen;
//long calDisColorGray;
//long calDisColorBlack;
//long calDisColorYellow;
//long calDisColorMagenta;
//long calDisColorCyan;
//long calDisColorWhite;
ColorSensorLib::ColorSensorLib(int r, int b, int g){
  r = r;
  b = b;
  g = g;
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  byte gammatable[256];
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
}

String returnColor(int r, int g, int b){
  long d1 = r - myblue[0];
  d1 = d1 * d1;
  //g - myblue.green
  long d2 = g - myblue[1];
  d2 = d2 * d2;
  //b - myblue.blue
  long d3 = b - myblue[2];
  d3 = d3 * d3;
  long distBlue = sqrt(d1 + d2 + d3);

  long d4 = r - myred[0];
  d4 = d4 * d4;
  //g - myblue.green
  long d5 = g - myred[1];
  d5 = d5 * d5;
  //b - myblue.blue
  long d6 = b - myred[2];
  d6 = d6 * d6;
  long distRed = sqrt(d4 + d5 + d6);

  long d7 = r - gray[0];
  d7 = d7 * d7;
  //g - myblue.green
  long d8 = g - gray[1];
  d8 = d8 * d8;
  //b - myblue.blue
  long d9 = b - gray[2];
  d9 = d9 * d9;
  long distGreen = sqrt(d7 + d8 + d9);


  long d10 = r - yellow[0];
  d10 = d10 * d10;
  //g - myblue.green
  long d11 = g - yellow[1];
  d11 = d11 * d11;
  //b - myblue.blue
  long d12 = b - yellow[2];
  d12 = d12 * d12;
  long distYellow = sqrt(d10 + d11 + d12);
  
  long d13 = r - cyan[0];
  d13 = d13 * d13;
  //g - myblue.green
  long d14 = g - cyan[1];
  d14 = d14 * d14;
  //b - myblue.blue
  long d15 = b - cyan[2];
  d15 = d15 * d15;
  long distCyan = sqrt(13 + d14 + d15);

   long d16 = r - magenta[0];
  d16 = d16 * d16;
  //g - myblue.green
  long d17 = g - magenta[1];
  d17 = d17 * d17;
  //b - myblue.blue
  long d18 = b - magenta[2];
  d18 = d18 * d18;
  long distMagenta = sqrt(d16 + d17 + d18);

   long d19 = r - black[0];
  d19 = d19 * d19;
  //g - myblue.green
  long d20 = g - black[1];
  d20 = d20 * d20;
  //b - myblue.blue
  long d21 = b - black[2];
  d21 = d21 * d21;
  long distBlack = sqrt(d19 + d20 + d21);

   long d23 = r - white[0];
  d23 = d23 * d23;
  //g - myblue.green
  long d24 = g - white[1];
  d24 = d24 * d24;
  //b - myblue.blue
  long d22 = b - white[2];
  d22 = d22 * d22;
  long distWhite = sqrt(d23 + d24 + d22);

   long d25 = r - gray[0];
  d25 = d25 * d25;
  //g - myblue.green
  long d26 = g - gray[1];
  d26 = d26 * d26;
  //b - myblue.blue
  long d27 = b - gray[2];
  d27 = d27 * d27;
  long distGray = sqrt(d25 + d26 + d27);
  /////////////////////////////////////////
  if (minOfAll > distBlue) {
    minOfAll = distBlue;
    minString = "blue";
  }
  
  
  if (minOfAll > distRed) {
    minOfAll = distRed;
    minString = "red";
  }

//  long distGreen = calDisColorGreen(r, g, b);
  if (minOfAll > distGreen) {
    minOfAll = distGreen;
    minString = "green";
  }
//  long distCyan = calDisColorCyan(r, g, b);
  if (minOfAll > distCyan) {
    minOfAll = distCyan;
    minString = "cyan";
  }
//  long distYellow = calDisColorYellow(r, g, b);
  if (minOfAll > distYellow) {
    minOfAll = distYellow;
    minString = "yellow";
  }
//  long distMagenta = calDisColorMagenta(r, g, b);
  if (minOfAll > distMagenta) {
    minOfAll = distMagenta;
    minString = "magenta";
  }
//  long distGray = calDisColorGray(r, g, b);
  if (minOfAll > distGray) {
    minOfAll = distGray;
    minString = "gray";
  }
//  long distBlack = calDisColorBlack(r, g, b);
  if (minOfAll > distBlack) {
    minOfAll = distBlack;
    minString = "black";
  }

//  long distWhite = calDisColorWhite(r, g, b);
  if (minOfAll > distWhite) {
    minOfAll = distWhite;
    minString = "white";
  }
  return minString;
}

//long calDisColorRed(int r, int g, int b) {
//  //r - myred.red
//  long d1 = (r - myred[0]);
//  d1 = d1 * d1;
//  //g - myred.green
//  long d2 = (g - myred[1]);
//  d2 = d2 * d2;
//  //b - myred.blue
//  long d3 = (b - myred[2]);
//  d3 = d3 * d3;
//  return sqrt(d1 + d2 + d3);
//}
//
//long calDisColorBlue(int r, int g, int b) {
//  //r - myblue.red
//  long d1 = r - myblue[0];
//  d1 = d1 * d1;
//  //g - myblue.green
//  long d2 = g - myblue[1];
//  d2 = d2 * d2;
//  //b - myblue.blue
//  long d3 = b - myblue[2];
//  d3 = d3 * d3;
//  return sqrt(d1 + d2 + d3);
//}
//
//long calDisColorGreen(int r, int g, int b) {
//  //r - mygreen.red
//  long d1 = r - mygreen[0];
//  d1 = d1 * d1;
//  //g - mygreen.green
//  long d2 = g - mygreen[1];
//  d2 = d2 * d2;
//  //b - mygreen.blue
//  long d3 = b - mygreen[2];
//  d3 = d3 * d3;
//  return sqrt(d1 + d2 + d3);
//}
//
//long calDisColorYellow(int r, int g, int b) {
//  //r - yellow.red
//  long d1 = r - yellow[0];
//  d1 = d1 * d1;
//  //g - yellow.green
//  long d2 = g - yellow[1];
//  d2 = d2 * d2;
//  //b - yellow.blue
//  long d3 = b - yellow[2];
//  d3 = d3 * d3;
//  return sqrt(d1 + d2 + d3);
//}
//
//long calDisColorCyan(int r, int g, int b) {
//  //r - cyan.red
//  long d1 = r - cyan[0];
//  d1 = d1 * d1;
//  //g - cyan.green
//  long d2 = g - cyan[1];
//  d2 = d2 * d2;
//  //b - cyan.blue
//  long d3 = b - cyan[2];
//  d3 = d3 * d3;
//  return sqrt(d1 + d2 + d3);
//}
//
//long calDisColorMagenta(int r, int g, int b) {
//  //r - magenta.red
//  long d1 = r - magenta[0];
//  d1 = d1 * d1;
//  //g - myred.green
//  long d2 = g - magenta[1];
//  d2 = d2 * d2;
//  //b - myred.blue
//  long d3 = b - magenta[2];
//  d3 = d3 * d3;
//  return sqrt(d1 + d2 + d3);
//}
//
//long calDisColorGray(int r, int g, int b) {
//  //r - magenta.red
//  long d1 = r - gray[0];
//  d1 = d1 * d1;
//  //g - myred.green
//  long d2 = g - gray[1];
//  d2 = d2 * d2;
//  //b - myred.blue
//  long d3 = b - gray[2];
//  d3 = d3 * d3;
//  return sqrt(d1 + d2 + d3);
//}
//
//long calDisColorBlack(int r, int g, int b) {
//  //r - black.red
//  long d1 = r - black[0];
//  d1 = d1 * d1;
//  //g - myred.green
//  long d2 = g - black[1];
//  d2 = d2 * d2;
//  //b - myred.blue
//  long d3 = b - black[2];
//  d3 = d3 * d3;
//  return sqrt(d1 + d2 + d3);
//}
//
//long calDisColorWhite(int r, int g, int b) {
//  //r - black.red
//  long d1 = r - white[0];
//  d1 = d1 * d1;
//  //g - myred.green
//  long d2 = g - white[1];
//  d2 = d2 * d2;
//  //b - myred.blue
//  long d3 = b - white[2];
//  d3 = d3 * d3;
//  return sqrt(d1 + d2 + d3);  
//}


