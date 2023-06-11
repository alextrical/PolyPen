#include "generated/PolyPen_menu.h"

void setup() {
    Wire.begin();
    setupMenu();

    //Use endstop pins for encoder
    // switches.init(internalDigitalIo(), SWITCHES_NO_POLLING, true);
    // menuMgr.initForEncoder(&renderer, &menuRun, PB5, PB7, PB6);
    switches.init(ioexp_io8574, SWITCHES_NO_POLLING, true);
    menuMgr.initForEncoder(&renderer, &menuRun, 7, 6, 5);

}

void loop() {
    taskManager.runLoop();

}


void CALLBACK_FUNCTION onRun(int id) {
    // TODO - your menu change code
}
