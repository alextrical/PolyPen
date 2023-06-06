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
const SubMenuInfo minfoAdvanced = { TC_I18N_MENU_14_NAME, 14, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackAdvanced(&minfoAdvanced, nullptr, INFO_LOCATION_PGM);
SubMenuItem menuAdvanced(&minfoAdvanced, &menuBackAdvanced, nullptr, INFO_LOCATION_PGM);
const BooleanMenuInfo minfoDedge = { TC_I18N_MENU_13_NAME, 13, 0xffff, 1, NO_CALLBACK, NAMING_TRUE_FALSE };
BooleanMenuItem menuDedge(&minfoDedge, false, nullptr, INFO_LOCATION_PGM);
const AnalogMenuInfo minfoHstrt = { TC_I18N_MENU_12_NAME, 12, 0xffff, 5, NO_CALLBACK, 0, 1, TC_I18N_MENU_12_UNIT };
AnalogMenuItem menuHstrt(&minfoHstrt, 1, &menuDedge, INFO_LOCATION_PGM);
const AnalogMenuInfo minfoHend = { TC_I18N_MENU_11_NAME, 11, 0xffff, 5, NO_CALLBACK, 0, 1, TC_I18N_MENU_11_UNIT };
AnalogMenuItem menuHend(&minfoHend, 2, &menuHstrt, INFO_LOCATION_PGM);
const AnalogMenuInfo minfoToff = { TC_I18N_MENU_10_NAME, 10, 0xffff, 5, NO_CALLBACK, 0, 1, TC_I18N_MENU_10_UNIT };
AnalogMenuItem menuToff(&minfoToff, 4, &menuHend, INFO_LOCATION_PGM);
const BooleanMenuInfo minfoReverse = { TC_I18N_MENU_9_NAME, 9, 0xffff, 1, NO_CALLBACK, NAMING_TRUE_FALSE };
BooleanMenuItem menuReverse(&minfoReverse, false, &menuToff, INFO_LOCATION_PGM);
const BooleanMenuInfo minfoInterpolate = { TC_I18N_MENU_8_NAME, 8, 0xffff, 1, NO_CALLBACK, NAMING_TRUE_FALSE };
BooleanMenuItem menuInterpolate(&minfoInterpolate, true, &menuReverse, INFO_LOCATION_PGM);
const AnalogMenuInfo minfoHoldMultiplier = { TC_I18N_MENU_7_NAME, 7, 0xffff, 100, NO_CALLBACK, 0, 100, TC_I18N_MENU_7_UNIT };
AnalogMenuItem menuHoldMultiplier(&minfoHoldMultiplier, 100, &menuInterpolate, INFO_LOCATION_PGM);
const char enumStrMicrostep_0[] = TC_I18N_MENU_6_ENUM_0;
const char enumStrMicrostep_1[] = TC_I18N_MENU_6_ENUM_1;
const char enumStrMicrostep_2[] = TC_I18N_MENU_6_ENUM_2;
const char enumStrMicrostep_3[] = TC_I18N_MENU_6_ENUM_3;
const char enumStrMicrostep_4[] = TC_I18N_MENU_6_ENUM_4;
const char enumStrMicrostep_5[] = TC_I18N_MENU_6_ENUM_5;
const char enumStrMicrostep_6[] = TC_I18N_MENU_6_ENUM_6;
const char enumStrMicrostep_7[] = TC_I18N_MENU_6_ENUM_7;
const char enumStrMicrostep_8[] = TC_I18N_MENU_6_ENUM_8;
const char* const enumStrMicrostep[]  = { enumStrMicrostep_0, enumStrMicrostep_1, enumStrMicrostep_2, enumStrMicrostep_3, enumStrMicrostep_4, enumStrMicrostep_5, enumStrMicrostep_6, enumStrMicrostep_7, enumStrMicrostep_8 };
const EnumMenuInfo minfoMicrostep = { TC_I18N_MENU_6_NAME, 6, 0xffff, 8, NO_CALLBACK, enumStrMicrostep };
EnumMenuItem menuMicrostep(&minfoMicrostep, 8, &menuHoldMultiplier, INFO_LOCATION_PGM);
const AnalogMenuInfo minfoCurrent = { TC_I18N_MENU_5_NAME, 5, 0xffff, 1000, NO_CALLBACK, 0, 1, TC_I18N_MENU_5_UNIT };
AnalogMenuItem menuCurrent(&minfoCurrent, 400, &menuMicrostep, INFO_LOCATION_PGM);
const BooleanMenuInfo minfoStealth = { TC_I18N_MENU_4_NAME, 4, 0xffff, 1, NO_CALLBACK, NAMING_TRUE_FALSE };
BooleanMenuItem menuStealth(&minfoStealth, true, &menuCurrent, INFO_LOCATION_PGM);
const SubMenuInfo minfoMotor = { TC_I18N_MENU_3_NAME, 3, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackMotor(&minfoMotor, &menuStealth, INFO_LOCATION_PGM);
SubMenuItem menuMotor(&minfoMotor, &menuBackMotor, &menuAdvanced, INFO_LOCATION_PGM);
const SubMenuInfo minfoSettings = { TC_I18N_MENU_2_NAME, 2, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackSettings(&minfoSettings, &menuMotor, INFO_LOCATION_PGM);
SubMenuItem menuSettings(&minfoSettings, &menuBackSettings, nullptr, INFO_LOCATION_PGM);
const BooleanMenuInfo minfoRun = { TC_I18N_MENU_1_NAME, 1, 0xffff, 1, onRun, NAMING_ON_OFF };
BooleanMenuItem menuRun(&minfoRun, false, &menuSettings, INFO_LOCATION_PGM);

void setupMenu() {
    // First we set up eeprom and authentication (if needed).
    setSizeBasedEEPROMStorageEnabled(true);
    menuMgr.setEepromRef(&glArduinoEeprom);
    // Now add any readonly, non-remote and visible flags.
    menuAdvanced.setSecured(true);
    menuInterpolate.setVisible(false);
    menuHstrt.setVisible(false);
    menuHoldMultiplier.setVisible(false);
    menuHend.setVisible(false);
    menuDedge.setVisible(false);
    menuToff.setVisible(false);

    // Code generated by plugins.
    gfx.begin();
    renderer.setUpdatesPerSecond(10);
    menuMgr.initWithoutInput(&renderer, &menuRun);
    renderer.setTitleMode(BaseGraphicalRenderer::TITLE_FIRST_ROW);
    renderer.setUseSliderForAnalog(false);
    installMonoBorderedTheme(renderer, MenuFontDef(nullptr, 1), MenuFontDef(nullptr, 1), true);
}

