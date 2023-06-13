#include "thermistor.h"

// Thermistor::Thermistor() {
// }

// ~~~~~ Thermistor Logic ~~~~~

/* ~~~~~~~~~~~ Median sorting function ~~~~~~~~~~~ */
// by Emilian Robert Vicol , https://robertvicol.com/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
void Thermistor::isort(int *a, int n) {
  for (int i = 1; i < n; ++i) {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--) {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}

/* ~~~~ analog reading median value from port ~~~~ */
// by Emilian Robert Vicol , https://robertvicol.com/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
int Thermistor::analogReadMedian(int port) {
  int values[READ_MEDIAN_COUNT];
  for (int i = 0; i != READ_MEDIAN_COUNT; ++i) {
    values[i] = analogRead(port);
    delay(READ_MEDIAN_DELAY);
  }
  isort(values, READ_MEDIAN_COUNT);
  return values[READ_MEDIAN_COUNT / 2 + 1];
}

/* ~~~~~~~~~~ Steinhart–Hart thermistor ~~~~~~~~~~ */
// by Emilian Robert Vicol , https://robertvicol.com/
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// TPin = Analog Pin
// THERMISTOR = NTC nominal value that is measured at 25*C
// R1 = R Serries
// A, B , C = the Steinhart–Hart coefficients, which vary depending on the type and model of thermistor and the temperature range of interest.
float Thermistor::f_ReadTemp_ThABC(int TPin, long THERMISTOR, float R1, float A, float B, float C) {

  int Vo = analogReadMedian(TPin);
  float Resistance = (65535 / (float)Vo) - 1;  // for pull-up configuration
  float R2 = R1 / Resistance;

  float logR2 = log(R2);  // Pre-Calcul for Log(R2)
  float T = (1.0 / (A + B * logR2 + C * logR2 * logR2 * logR2));
  T = T - 273.15;  // convert Kelvin to *C
  return T;
}