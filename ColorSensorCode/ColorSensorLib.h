#ifndef ColorSensorlib
#define ColorSensorlib
#include "Arduino.h"

class ColorSensorLib{
  public:
    ColorSensorLib(int r, int b, int g);
    String returnColor(int minOfAll);
//    long calDisColorRed(int cR, int cG, int cB);
//    long calDisColorBlue(int cR, int cG, int cB);
//    long calDisColorGreen(int cR, int cG, int cB);
//    long calDisColorYellow(int cR, int cG, int cB);
//    long calDisColorCyan(int cR, int cG, int cB);
//    long calDisColorMagenta(int cR, int cG, int cB);
//    long calDisColorGray(int cR, int cG, int cB);
//    long calDisColorBlack(int cR, int cG, int cB);
//    long calDisColorWhite(int cR, int cG, int cB);

};
#endif

