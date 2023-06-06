#include "generated/PolyPen_menu.h"
#include <IWatchdog.h> //https://github.com/stm32duino/Arduino_Core_STM32/tree/main/libraries/IWatchdog

void setup() {
    // Initialize the IWDG with 4 seconds timeout.
    // This would cause a CPU reset if the IWDG timer
    // is not reloaded in approximately 4 seconds.
    IWatchdog.begin(4000000);
    
    Wire.begin();
    setupMenu();

}

void loop() {
    IWatchdog.reload();
    taskManager.runLoop();

}


void CALLBACK_FUNCTION onRun(int id) {
    // TODO - your menu change code
}
