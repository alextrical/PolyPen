#ifndef MOTOR_H
#define MOTOR_H
#include "Arduino.h"
#include <TMCStepper.h>

#define EN_PIN           PD2 // Enable
#define DIR_PIN          PD1 // Direction
#define STEP_PIN         PD0 // Step
#define DRIVER_ADDRESS  0b00 // TMC2209 Driver address according to MS1 and MS2

#define TMC_BAUD_RATE    115200

#define R_SENSE 0.11f // Match to your driver
                      // BTT EBB42/36 use 0.11

class Motor {
  public:
    bool stealth = true;
    bool reverse = true;
    bool interpolate = true;
    uint16_t mA = 200;
    uint16_t microsteps = 64;
    float stepsPerMm = 200;
    float mmPerSec = 1;
    float hold_multiplier = 1;

    void begin();
    void stepperSet();
  private:
};

#endif