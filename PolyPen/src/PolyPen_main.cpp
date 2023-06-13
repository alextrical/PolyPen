#include "generated/PolyPen_menu.h"
#include "module/thermistor.h"
#include "module/motor.h"


#include <IoAbstraction.h>
#include <TaskManagerIO.h>

#if defined(STM32_CORE_VERSION) && (STM32_CORE_VERSION  < 0x01090000)
//STM32 HW - Need to detect Compiler HW and switch to matching
#include <IWatchdog.h> //https://github.com/stm32duino/Arduino_Core_STM32/tree/main/libraries/IWatchdog
#elif !defined(STM32_CORE_VERSION)
//Add other Watchdog stuff here
#endif

#if defined(LED_BUILTIN) && !defined(pin)
#define pin  LED_BUILTIN
#else
#define pin  D2
#endif

bool button1Pressed = false;
//The pin onto which we connected the rotary encoders switch
const pinid_t BUTTON1_PIN = PB8;

#define HEATER_PIN PB13 //PA2
#define HEATER2_PIN PA2

#define HEATER_MINTEMP        5
#define HEATER_MAXTEMP      275
#define HEATER_OVERSHOOT     15   // (°C) Forbid temperatures over MAXTEMP - OVERSHOOT
#define HEATER_FAULT_VALIDATION_TIME 5    // number of 1/10 second loops before signalling a heater fault

//~~~PID Heating~~~
#include <QuickPID.h>

//Define Variables we'll be connecting to
float Setpoint = 100, Input, Output;

float Kp = 4, Ki = 0.2, Kd = 1;

//specify the links
QuickPID myPID(&Input, &Output, &Setpoint);
Motor motor = Motor();
Thermistor thermistor = Thermistor();

// ~~~Quality checks~~~
#ifndef HEATER_MINTEMP
#define HEATER_MINTEMP 5
#endif
#ifndef HEATER_MAXTEMP
#define HEATER_MAXTEMP 275
#endif
#ifndef HEATER_OVERSHOOT
#define HEATER_OVERSHOOT 15
#endif
#define HEATER_MAX_TARGET (HEATER_MAXTEMP - (HEATER_OVERSHOOT))

void setup() {
    Serial.begin(9600);
    #if defined(STM32_CORE_VERSION) && (STM32_CORE_VERSION  < 0x01090000)
        // Initialize the IWDG with 4 seconds timeout.
        // This would cause a CPU reset if the IWDG timer
        // is not reloaded in approximately 4 seconds.
        IWatchdog.begin(4000000); //STM32
        taskManager.scheduleFixedRate(1000, [] { // schedule tasks to run at a fixed rate, every 1000 milliseconds.
            IWatchdog.reload(); //STM32
        });
    #endif

    pinMode(HEATER_PIN, OUTPUT);
    #ifdef HEATER_PIN2
    pinMode(HEATER2_PIN, OUTPUT);
    #endif

    pinMode(pin, OUTPUT);
    taskManager.scheduleFixedRate(500, [] { // schedule tasks to run at a fixed rate, every 500 milliseconds.
        menuTemperature.setFloatValue(thermistor.f_ReadTemp_ThABC(THERMISTOR_PIN, 100000, 4700, TA, TB, TC));
        digitalWrite(pin, !digitalRead(pin));
    });

    Wire.begin();
    motor.begin();
    thermistor.begin();

    //~~~~Menu Setup~~~~
    setupMenu();

    // Use endstop pins for encoder, Define here so we can eventually detect and set the source type automatically
    switches.init(internalDigitalIo(), SWITCHES_NO_POLLING, true);
    menuMgr.initForEncoder(&renderer, &menuRun, PB7, PB5, PB6);
    // switches.init(ioexp_io8574, SWITCHES_NO_POLLING, true);
    // menuMgr.initForEncoder(&renderer, &menuRun, 7, 6, 5);

    //~~~PID Heating~~~
    //apply PID gains
    myPID.SetTunings(Kp, Ki, Kd);

    //turn the PID on
    myPID.SetMode(myPID.Control::automatic);


    taskManager.scheduleFixedRate(100, [] { // schedule tasks to run at a fixed rate, every 100 milliseconds.
        // bool running = menuRun.getBoolean();
        if (menuRun.getBoolean()) {
            Input = thermistor.f_ReadTemp_ThABC(THERMISTOR_PIN, 100000, 4700, TA, TB, TC);
            float gap = Setpoint - Input; //distance away from setpoint
            // if (gap > -10) { //we're close to melt temperature, enable the extruder motor
            if (Input > 120 && button1Pressed) { //we're close to melt temperature, enable the extruder motor
                //run motor
                digitalWrite(EN_PIN, LOW);
            } else {
                digitalWrite(EN_PIN, HIGH);
            }
            myPID.Compute();
            if (gap < -10) {
                Output = 0;
            } else if (gap > 10) {
                Output = 255;
            }


            // If the heater temp is abnormal, confirm state before signaling panel
            uint8_t faultDuration = 0;
            // while (!WITHIN(temperature, HEATER_MINTEMP, HEATER_MAXTEMP)) {
            while (Input<HEATER_MINTEMP||Input>HEATER_MAXTEMP) {
                faultDuration++;
                Input = thermistor.f_ReadTemp_ThABC(THERMISTOR_PIN, 100000, 4700, TA, TB, TC);  // 100k thermistor; 4.7K pullup resistor; ABC parameters were calculated using the datasheet!
                if (faultDuration >= HEATER_FAULT_VALIDATION_TIME) {
                //SendtoTFTLN(AC_msg_nozzle_temp_abnormal);
                // last_error(AC_error_abnormal_temp_bed);
                // last_error currentState = AC_error_abnormal_temp_bed;
                Serial.print("Bed temp abnormal! : ");
                Serial.print(Input);
                Serial.println("C");
                Output = 0;
                break;
                }
                delay(100); //wait 0.1 seconds
            }
        } else {
            Output = 0; 
        }
        analogWrite(HEATER_PIN, Output);
        #ifdef HEATER_PIN2
        analogWrite(HEATER2_PIN, Output);
        #endif
    });

    // switches.addSwitch(BUTTON1_PIN, onButton1Clicked, NO_REPEAT);
    // switches.onRelease(BUTTON1_PIN, onButton1Released);

    switches.addSwitch(BUTTON1_PIN, [](pintype_t , bool held) {
        Serial.print("Button 1 pressed");
        button1Pressed = true;
        Serial.println(held ? " Held" : " Pressed");
    }, NO_REPEAT);
    switches.onRelease(BUTTON1_PIN, [](pintype_t , bool) {
        Serial.print("Button 1 released");
        button1Pressed = false;
    });
}

void loop() {
    taskManager.runLoop();
}


void CALLBACK_FUNCTION onRun(int id) {
    // TODO - your menu change code
    onStepperSet(0);
}


void CALLBACK_FUNCTION onSetpointChange(int id) {
    // TODO - your menu change code
    Setpoint = menuSetpoint.getAsFloatingPointValue();
}


void CALLBACK_FUNCTION onStepperSet(int id) {
    motor.stealth = menuStealth.getBoolean();
    motor.mA = menuCurrent.getCurrentValue();
    int stepChoice = menuMicrostep.getCurrentValue();
    switch (stepChoice) {
        case 0:
            motor.microsteps = 1;
            break;
        case 1:
            motor.microsteps = 2;
            break;
        case 2:
            motor.microsteps = 4;
            break;
        case 3:
            motor.microsteps = 8;
            break;
        case 4:
            motor.microsteps = 16;
            break;
        case 5:
            motor.microsteps = 32;
            break;
        case 6:
            motor.microsteps = 64;
            break;
        case 7:
            motor.microsteps = 128;
            break;
        case 8:
        default:
            motor.microsteps = 256;
            break;
    }
    motor.interpolate = menuInterpolate.getBoolean();
    motor.reverse = menuReverse.getBoolean();
    motor.stepsPerMm = 200;
    motor.mmPerSec = menuSpeed.getAsFloatingPointValue();
    motor.stepperSet();
    thermistor.meltzoneFanTemp = 40;
}