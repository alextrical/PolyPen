#ifndef THERMISTOR_H
#define THERMISTOR_H
#include "Arduino.h"

class Thermistor {
  public:
    float f_ReadTemp_ThABC(int TPin, long THERMISTOR, float R1, float A, float B, float C);
  private:
    void isort(int *a, int n);
    int analogReadMedian(int port);
    int READ_MEDIAN_COUNT = 15;
    int READ_MEDIAN_DELAY = 2;
};

#endif
