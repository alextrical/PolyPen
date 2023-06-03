#include "generated/PolyPen_menu.h"

void setup() {
    Wire.begin();
    setupMenu();

}

void loop() {
    taskManager.runLoop();

}


void CALLBACK_FUNCTION onStart(int id) {
    // TODO - your menu change code
}
