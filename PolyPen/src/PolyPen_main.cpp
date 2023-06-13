#include "generated/PolyPen_menu.h"
#include <TMCStepper.h>
#include "module/thermistor.h"

#include "TaskManagerIO.h"

//STM32 HW - Need to detect Compiler HW and switch to matching
#include <IWatchdog.h> //https://github.com/stm32duino/Arduino_Core_STM32/tree/main/libraries/IWatchdog


//~~~~Thermistor setup~~~
Thermistor thermistor = Thermistor();
#define THERMISTOR_PIN PA3


#define TA 0.7899581069E-3
#define TB 2.153376487E-4
#define TC 0.4538837458E-7  


//~~~~Stepper setup~~~~
HardwareSerial Serial2(PA15, PA14_ALT1);

#define EN_PIN           PD2 // Enable
#define DIR_PIN          PD1 // Direction
#define STEP_PIN         PD0 // Step
#define DRIVER_ADDRESS  0b00 // TMC2209 Driver address according to MS1 and MS2

#define TMC_BAUD_RATE    115200
bool stealth = true;
uint16_t mA = 200;
uint16_t microsteps = 64;
float hold_multiplier = 1;
float stepsPerMm = 200;
float mmPerSec = 1;
bool interpolate = true;
bool reverse = true;
// uint32_t HYBRID_THRESHOLD = 30;

#define R_SENSE 0.11f // Match to your driver
                      // BTT EBB42/36 use 0.11

// Select your stepper driver type
TMC2209Stepper driver(&Serial2, R_SENSE, DRIVER_ADDRESS);       // Hardware Serial
//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);     // Software serial


void stepperSet(){
    driver.shaft(reverse); //Set motor direction in software
    driver.en_spreadCycle(!stealth);
    driver.intpol(interpolate);
    driver.rms_current(mA, hold_multiplier); // Set motor RMS current
    driver.microsteps(microsteps); // Set microsteps

    driver.VACTUAL((microsteps*stepsPerMm*mmPerSec)); //Set the frequency of the oscilator
}

void setup() {
    // Initialize the IWDG with 4 seconds timeout.
    // This would cause a CPU reset if the IWDG timer
    // is not reloaded in approximately 4 seconds.
    IWatchdog.begin(4000000); //STM32
    taskManager.scheduleFixedRate(1000, [] { // schedule tasks to run at a fixed rate, every 1000 milliseconds.
        IWatchdog.reload(); //STM32
    });

    taskManager.scheduleFixedRate(500, [] { // schedule tasks to run at a fixed rate, every 500 milliseconds.
        menuTemperature.setFloatValue(thermistor.f_ReadTemp_ThABC(THERMISTOR_PIN, 100000, 4700, TA, TB, TC));
    });

    Wire.begin();

    //~~~~Menu Setup~~~~
    setupMenu();

    // Use endstop pins for encoder, Define here so we can eventually detect and set the source type automatically
    switches.init(internalDigitalIo(), SWITCHES_NO_POLLING, true);
    menuMgr.initForEncoder(&renderer, &menuRun, PB7, PB5, PB6);
    // switches.init(ioexp_io8574, SWITCHES_NO_POLLING, true);
    // menuMgr.initForEncoder(&renderer, &menuRun, 7, 6, 5);

    //~~Thermistor Setup~~~
    pinMode(THERMISTOR_PIN, INPUT);
    analogReadResolution(16);  //https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogreadresolution/


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

void loop() {
    taskManager.runLoop();
}


void CALLBACK_FUNCTION onRun(int id) {
    // TODO - your menu change code
    digitalWrite(EN_PIN, !digitalRead(EN_PIN));
}


void CALLBACK_FUNCTION onSetpointChange(int id) {
    // TODO - your menu change code
}


void CALLBACK_FUNCTION onStepperSet(int id) {
    stealth = menuStealth.getBoolean();
    mA = menuCurrent.getCurrentValue();
    int stepChoice = menuMicrostep.getCurrentValue();
    switch (stepChoice) {
        case 0:
            microsteps = 1;
            break;
        case 1:
            microsteps = 2;
            break;
        case 2:
            microsteps = 4;
            break;
        case 3:
            microsteps = 8;
            break;
        case 4:
            microsteps = 16;
            break;
        case 5:
            microsteps = 32;
            break;
        case 6:
            microsteps = 64;
            break;
        case 7:
            microsteps = 128;
            break;
        case 8:
        default:
            microsteps = 256;
            break;
    }
    interpolate = menuInterpolate.getBoolean();
    reverse = menuReverse.getBoolean();
    stepsPerMm = 200;
    mmPerSec = menuSpeed.getAsFloatingPointValue();
    stepperSet();
}
