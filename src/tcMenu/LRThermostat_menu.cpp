/*
    The code in this file uses open source libraries provided by thecoderscorner

    DO NOT EDIT THIS FILE, IT WILL BE GENERATED EVERY TIME YOU USE THE UI DESIGNER
    INSTEAD EITHER PUT CODE IN YOUR SKETCH OR CREATE ANOTHER SOURCE FILE.

    All the variables you may need access to are marked extern in this file for easy
    use elsewhere.
 */

#include <tcMenu.h>
#include "LRThermostat_menu.h"
#include "ThemeCoolBlueTraditional.h"
#include "LRThermostat.h"

// Global variable declarations
const PROGMEM  ConnectorLocalInfo applicationInfo = { "Thermostat Menu", "a4a49d27-69e5-459b-a2ea-e3e521bd54d2" };
ArduinoEEPROMAbstraction glArduinoEeprom(&EEPROM);
TFT_eSPI tft;
TfteSpiDrawable tftDrawable(&tft, 2);
GraphicsDeviceRenderer renderer(30, applicationInfo.name, &tftDrawable);

// Global Menu Item declarations
const PROGMEM AnyMenuInfo minfoSafeShutdown = { "Safe Shutdown", 78, 0xffff, 0, SafeShutdown };
ActionMenuItem menuSafeShutdown(&minfoSafeShutdown, NULL);
RENDERING_CALLBACK_NAME_INVOKE(fnBaroSteadyUpLimitRtCall, largeNumItemRenderFn, "Pr Steady", 64, BaroSteadyUpLimitCallback)
EditableLargeNumberMenuItem menuBaroSteadyUpLimit(fnBaroSteadyUpLimitRtCall, LargeFixedNumber(5, 4, 0U, 80U, false), 92, false, NULL);
const PROGMEM AnalogMenuInfo minfoMinRunTime = { "DH Min RT", 91, 10, 180, MinRunTimeCallback, 0, 1, " min" };
AnalogMenuItem menuMinRunTime(&minfoMinRunTime, 30, &menuBaroSteadyUpLimit);
RENDERING_CALLBACK_NAME_INVOKE(fnMiscellaneousRtCall, backSubItemRenderFn, "Miscellaneous", -1, NO_CALLBACK)
const PROGMEM SubMenuInfo minfoMiscellaneous = { "Miscellaneous", 90, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackMiscellaneous(fnMiscellaneousRtCall, &menuMinRunTime);
SubMenuItem menuMiscellaneous(&minfoMiscellaneous, &menuBackMiscellaneous, NULL);
RENDERING_CALLBACK_NAME_INVOKE(fnPressureCalRtCall, largeNumItemRenderFn, "Pressure", 48, PressureCalCallback)
EditableLargeNumberMenuItem menuPressureCal(fnPressureCalRtCall, LargeFixedNumber(4, 2, 0U, 0U, false), 85, true, NULL);
RENDERING_CALLBACK_NAME_INVOKE(fnTemperatureCalRtCall, largeNumItemRenderFn, "Temperature", 32, TempCalCallback)
EditableLargeNumberMenuItem menuTemperatureCal(fnTemperatureCalRtCall, LargeFixedNumber(4, 2, 0U, 0U, false), 84, true, &menuPressureCal);
RENDERING_CALLBACK_NAME_INVOKE(fnHumidityCalRtCall, largeNumItemRenderFn, "Humidity", 16, HumidityCalCallback)
EditableLargeNumberMenuItem menuHumidityCal(fnHumidityCalRtCall, LargeFixedNumber(4, 2, 0U, 0U, false), 83, true, &menuTemperatureCal);
RENDERING_CALLBACK_NAME_INVOKE(fnSensorCalibrationRtCall, backSubItemRenderFn, "Sensor Calibration", -1, NO_CALLBACK)
const PROGMEM SubMenuInfo minfoSensorCalibration = { "Sensor Calibration", 82, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackSensorCalibration(fnSensorCalibrationRtCall, &menuHumidityCal);
SubMenuItem menuSensorCalibration(&minfoSensorCalibration, &menuBackSensorCalibration, &menuMiscellaneous);
RENDERING_CALLBACK_NAME_INVOKE(fnHumdHysteresisRtCall, largeNumItemRenderFn, "Dehumidifying", 24, HumdHysteresisCallback)
EditableLargeNumberMenuItem menuHumdHysteresis(fnHumdHysteresisRtCall, LargeFixedNumber(4, 2, 10U, 0U, false), 89, false, NULL);
RENDERING_CALLBACK_NAME_INVOKE(fnHeatingHysteresisRtCall, largeNumItemRenderFn, "Heating", 40, HeatingHysteresisCallback)
EditableLargeNumberMenuItem menuHeatingHysteresis(fnHeatingHysteresisRtCall, LargeFixedNumber(4, 2, 1U, 0U, false), 88, false, &menuHumdHysteresis);
RENDERING_CALLBACK_NAME_INVOKE(fnCoolingHysteresisRtCall, largeNumItemRenderFn, "Cooling", 56, CoolingHysteresisCallback)
EditableLargeNumberMenuItem menuCoolingHysteresis(fnCoolingHysteresisRtCall, LargeFixedNumber(4, 2, 1U, 0U, false), 87, false, &menuHeatingHysteresis);
RENDERING_CALLBACK_NAME_INVOKE(fnHysteresisRtCall, backSubItemRenderFn, "Hysteresis", -1, NO_CALLBACK)
const PROGMEM SubMenuInfo minfoHysteresis = { "Hysteresis", 86, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackHysteresis(fnHysteresisRtCall, &menuCoolingHysteresis);
SubMenuItem menuHysteresis(&minfoHysteresis, &menuBackHysteresis, &menuSensorCalibration);
RENDERING_CALLBACK_NAME_INVOKE(fnThermostatSettingsRtCall, backSubItemRenderFn, "Settings", -1, NO_CALLBACK)
const PROGMEM SubMenuInfo minfoThermostatSettings = { "Settings", 8, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackThermostatSettings(fnThermostatSettingsRtCall, &menuHysteresis);
SubMenuItem menuThermostatSettings(&minfoThermostatSettings, &menuBackThermostatSettings, &menuSafeShutdown);

const char enumStrFanEnum_0[] PROGMEM = "Auto";
const char enumStrFanEnum_1[] PROGMEM = "On";
const char enumStrFanEnum_2[] PROGMEM = "Dhm";
const char *const enumStrFanEnum[] PROGMEM = {enumStrFanEnum_0, enumStrFanEnum_1, enumStrFanEnum_2};

const PROGMEM EnumMenuInfo minfoFanEnum = {"Fan", 49, 4, 2, FanCallback, enumStrFanEnum};
EnumMenuItem menuFanEnum(&minfoFanEnum, 1, &menuThermostatSettings);
const char enumStrModeEnum_0[] PROGMEM = "Off";
const char enumStrModeEnum_1[] PROGMEM = "Heat";
const char enumStrModeEnum_2[] PROGMEM = "Cool";
const char enumStrModeEnum_3[] PROGMEM = "Dehumidify";
const char* const enumStrModeEnum[] PROGMEM  = { enumStrModeEnum_0, enumStrModeEnum_1, enumStrModeEnum_2, enumStrModeEnum_3 };
const PROGMEM EnumMenuInfo minfoModeEnum = { "Mode", 39, 6, 3, ModeCallback, enumStrModeEnum };
EnumMenuItem menuModeEnum(&minfoModeEnum, 0, &menuFanEnum);
const PROGMEM AnyMenuInfo minfoExitVar = { "Exit", 95, 0xffff, 0, ExitCallback };
ActionMenuItem menuExitVar(&minfoExitVar, &menuModeEnum);
const PROGMEM AnyMenuInfo minfoClearUsageCntrs = { "Clear Usage", 80, 0xffff, 0, ClearUsageCntrs };
ActionMenuItem menuClearUsageCntrs(&minfoClearUsageCntrs, NULL);
const PROGMEM AnyMenuInfo minfoDisplayUsageCntrs = { "Usage Counters", 81, 0xffff, 0, DisplayUsageCntrs };
ActionMenuItem menuDisplayUsageCntrs(&minfoDisplayUsageCntrs, &menuClearUsageCntrs);
const PROGMEM AnyMenuInfo minfoDisplaySysInfo = { "System Info", 99, 0xffff, 0, DisplaySysInfo };
ActionMenuItem menuDisplaySysInfo(&minfoDisplaySysInfo, &menuDisplayUsageCntrs);
const PROGMEM AnyMenuInfo minfoDisplayHmdGraph = { "Humidity Graph", 97, 0xffff, 0, DisplayHmdGraph };
ActionMenuItem menuDisplayHmdGraph(&minfoDisplayHmdGraph, &menuDisplaySysInfo);
const PROGMEM AnyMenuInfo minfoDisplayTempGraph = { "Temperature Graph", 98, 0xffff, 0, DisplayTempGraph };
ActionMenuItem menuDisplayTempGraph(&minfoDisplayTempGraph, &menuDisplayHmdGraph);
const PROGMEM AnyMenuInfo minfoDisplayBaroGraph = { "Baro Pres Graph", 96, 0xffff, 0, DisplayBaroGraph };
ActionMenuItem menuDisplayBaroGraph(&minfoDisplayBaroGraph, &menuDisplayTempGraph);
RENDERING_CALLBACK_NAME_INVOKE(fnUsageAndGraphsRtCall, backSubItemRenderFn, "Usage and Graphs", -1, NO_CALLBACK)
const PROGMEM SubMenuInfo minfoUsageAndGraphs = { "Usage and Graphs", 79, 0xffff, 0, NO_CALLBACK };
BackMenuItem menuBackUsageAndGraphs(fnUsageAndGraphsRtCall, &menuDisplayBaroGraph);
SubMenuItem menuUsageAndGraphs(&minfoUsageAndGraphs, &menuBackUsageAndGraphs, &menuExitVar);

void setupMenu() {
    // First we set up eeprom and authentication (if needed).
    menuMgr.setEepromRef(&glArduinoEeprom);
    // Code generated by plugins.
    // RSR: Don't re-init the display as it was already done by the splash
    // tft.begin();
    // tft.setRotation(3);
    renderer.setUpdatesPerSecond(10);
    switches.init(internalDigitalIo(), SWITCHES_POLL_EVERYTHING, true);
    // RSR: Update the following line with the switch number macros
    menuMgr.initForUpDownOk(&renderer, &menuUsageAndGraphs, DOWN_SWITCH, UP_SWITCH, ENTER_SWITCH, 20);
    renderer.setTitleMode(BaseGraphicalRenderer::TITLE_ALWAYS);
    renderer.setUseSliderForAnalog(false);
    installCoolBlueTraditionalTheme(renderer, MenuFontDef(nullptr, 2), MenuFontDef(nullptr, 1), true);
}
