#ifndef THERMISTOR_H
#define THERMISTOR_H
#include "Arduino.h"

class Thermistor {
  public:
    #define THERMISTOR_PIN PA3
    #define FAN_PIN PA0
    #define TA 0.7899581069E-3
    #define TB 2.153376487E-4
    #define TC 0.4538837458E-7 
    int meltzoneFanTemp = 40; //Temperature the fan will kick in at
    int fanSpeed = 100; //% speed for cooling the heatbreak
    int analogReadResolutionBits = 16;

    void begin();
    float f_ReadTemp_ThABC(int TPin, long THERMISTOR, float R1, float A, float B, float C);
  private:
    int READ_MEDIAN_COUNT = 15;
    int READ_MEDIAN_DELAY = 2;

    int analogReadMedian(int port);
    void isort(int *a, int n);
};

#endif
