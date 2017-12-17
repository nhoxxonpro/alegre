#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_Circuit_Playground.h>

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

//************************
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground
// set to false if using a common cathode LED
#define commonAnode true
// our RGB -> eye-recognized gamma color
byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
void setup() {
  //colorSetup();
  Serial.begin(9600);
  Serial.println("Color View Test!");
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  // use these three pins to drive an LED
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);


  }

}
void loop() {
  uint16_t clear, red, green, blue;
  tcs.setInterrupt(false);      // turn on LED
  delay(60);  // takes 50ms to read

  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);  // turn off LED

  /*Serial.print("C:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(red);
    Serial.print("\tG:\t"); Serial.print(green);
    Serial.print("\tB:\t"); Serial.print(blue);
  */
  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  Serial.print("\t");
  //Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX); Serial.print("\t");
  Serial.print((int)r ); Serial.print(" "); Serial.print((int)g); Serial.print(" ");  Serial.println((int)b );
  Serial.println();

  //Serial.print("Calibrated color:   ");Serial.print(redF);Serial.print("\t");Serial.print(blackF);Serial.print("\t");Serial.print(greenF);Serial.print("\t");Serial.print(whiteF);
  //Serial.print("\t");Serial.print(blueF); Serial.print("\t");Serial.print(grayF);Serial.print("\t");Serial.print(magentaF);Serial.print("\t");Serial.print(cyanF);Serial.print("\n");
  //float input = calDisColor(r,g,b);
  //Serial.print("Input ");Serial.print(input);Serial.print("\n");
  //Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );
  analogWrite(redpin, gammatable[(int)r]);
  analogWrite(greenpin, gammatable[(int)g]);
  analogWrite(bluepin, gammatable[(int)b]);


  long minOfAll = 1000;
  String minString;

  String returnColor(int minOfAll);

  Serial.print(distRed);
  Serial.print(" ");
  Serial.print(distBlue);
  Serial.print(" ");
  Serial.print(distGreen);
  Serial.print(" ");
  Serial.print(distCyan);
  Serial.print(" ");
  Serial.print(distYellow);
  Serial.print(" ");
  Serial.print(distMagenta);

  Serial.print(minString);//print the minString of all
}
//calculate the distance of the color based on calibrated color
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


