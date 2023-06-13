#include "motor.h"



//~~~~Stepper setup~~~~
HardwareSerial Serial2(PA15, PA14_ALT1);

// Select your stepper driver type
TMC2209Stepper driver(&Serial2, R_SENSE, DRIVER_ADDRESS);       // Hardware Serial
//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);     // Software serial


void Motor::begin(){
    //~~~~Stepper driver setup~~~~
    pinMode(EN_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);  //Enable driver in hardware, LOW is Enable
    digitalWrite(DIR_PIN, LOW); //LOW or HIGH
    digitalWrite(STEP_PIN, LOW);

    // Enable one according to your setup
    Serial2.begin(TMC_BAUD_RATE);          // HW UART drivers
    // driver.beginSerial(TMC_BAUD_RATE);     // SW UART drivers

    //Setup HW Serial for the TMC2209 on the EBB42/36 v1.1/v1.2 STM32G0B1CBT
    USART2->CR1 &= ~USART_CR1_UE;   // UE = 0... disable USART
    USART2->CR2 |= USART_CR2_SWAP;  // Swap TX/RX pins2
    USART2->CR3 |= USART_CR3_HDSEL; // Set Half-duplex selection
    USART2->CR1 |= USART_CR1_UE;    // UE = 1... Enable USART

    //~~~Setup Stepper registers~~~
    driver.pdn_disable(true); // Use UART
    driver.mstep_reg_select(true); // Select microsteps with UART
    driver.I_scale_analog(false);
    // driver.en_spreadCycle(!stealth);
    // driver.GCONF(gconf.sr);
    driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                    // UART: Init SW UART (if selected) with default 115200 baudrate
    driver.tbl(0b01); // blank_time = 24
    driver.toff(4); //CHOPPER_DEFAULT_24V
    // driver.intpol(interpolate);
    driver.hend(2 + 3); //CHOPPER_DEFAULT_24V
    driver.hstrt(1 - 1); //CHOPPER_DEFAULT_24V
    driver.dedge(false); //Enable to step on edge/state change
    // driver.CHOPCONF(chopconf.sr);
    // driver.rms_current(mA, hold_multiplier); // Set motor RMS current
    // driver.microsteps(microsteps); // Set microsteps
    // driver.shaft(reverse); //Set motor direction in software
    driver.iholddelay(10);
    driver.TPOWERDOWN(128); // ~2s until driver lowers to hold current
    driver.pwm_lim(12);
    driver.pwm_reg(8);
    driver.pwm_autograd(true);
    driver.pwm_autoscale(true);     // Needed for stealthChop
    driver.pwm_freq(0b01);
    driver.pwm_grad(14);
    driver.pwm_ofs(36);

    stepperSet();

    // driver.PWMCONF(driver.sr);
    // driver.set_pwm_thrs(HYBRID_THRESHOLD);
    driver.GSTAT(0b111); // Clear
    //delay(200);

    //driver.VACTUAL((microsteps*200));
}

void Motor::stepperSet(){
    driver.shaft(reverse); //Set motor direction in software
    driver.en_spreadCycle(!stealth);
    driver.intpol(interpolate);
    driver.rms_current(mA, hold_multiplier); // Set motor RMS current
    driver.microsteps(microsteps); // Set microsteps

    driver.VACTUAL((microsteps*stepsPerMm*mmPerSec)); //Set the frequency of the oscilator
}