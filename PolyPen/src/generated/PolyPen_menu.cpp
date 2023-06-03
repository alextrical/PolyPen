/*
    The code in this file uses open source libraries provided by thecoderscorner

    DO NOT EDIT THIS FILE, IT WILL BE GENERATED EVERY TIME YOU USE THE UI DESIGNER
    INSTEAD EITHER PUT CODE IN YOUR SKETCH OR CREATE ANOTHER SOURCE FILE.

    All the variables you may need access to are marked extern in this file for easy
    use elsewhere.
 */

#include <tcMenu.h>
#include "PolyPen_menu.h"
#include "../ThemeMonoBordered.h"

// Global variable declarations
const  ConnectorLocalInfo applicationInfo = { "PolyPen", "510b4fcf-cf16-4ba2-b736-cc56b6e828ea" };
ArduinoEEPROMAbstraction glArduinoEeprom(&EEPROM);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C gfx(U8G2_R0, PD3, U8X8_PIN_NONE, U8X8_PIN_NONE);
U8g2Drawable gfxDrawable(&gfx, &Wire);
GraphicsDeviceRenderer renderer(30, applicationInfo.name, &gfxDrawable);

// Global Menu Item declarations
const AnyMenuInfo minfoStart = { TC_I18N_MENU_1_NAME, 1, 0xffff, 0, onStart };
ActionMenuItem menuStart(&minfoStart, nullptr, INFO_LOCATION_PGM);

void setupMenu() {
    // First we set up eeprom and authentication (if needed).
    setSizeBasedEEPROMStorageEnabled(true);
    menuMgr.setEepromRef(&glArduinoEeprom);
    // Code generated by plugins.
    gfx.begin();
    renderer.setUpdatesPerSecond(10);
    switches.init(internalDigitalIo(), SWITCHES_NO_POLLING, true);
    menuMgr.initForEncoder(&renderer, &menuStart, PB5, PB7, PB6);
    renderer.setTitleMode(BaseGraphicalRenderer::TITLE_FIRST_ROW);
    renderer.setUseSliderForAnalog(false);
    installMonoBorderedTheme(renderer, MenuFontDef(nullptr, 1), MenuFontDef(nullptr, 1), true);
}

