// Copyright © 2022 Randy Rysavy <randy.rysavy@gmail.com>
// This work is free. You can redistribute it and/or modify it under the
// terms of the Do What The Fuck You Want To Public License, Version 2,
// as published by Sam Hocevar. See http://www.wtfpl.net/ for more details.

// Features to add / bugs to fix
// 1) DONE - clean up and break out "on time" counters
// 2) DONE - A way to clear the "on time" counters
// 3) DONE - An 'eject' like menu item that saves NVM stuff before power down
// 4) DONE - Change dehumidifier hysteresis so that it is +1N/-0N instead +0.5N/-0.5N.
//           We want to shut off at the specified humidity level.
// 5) DONE - Don't hang waiting for WiFi connection. Should be able to operate without WiFi.
// 6) DONE - Add barometric pressure to display
// -------------------------------------------------------------------------------------
// 7) Write main HTML status page including uptime and "on times"
// 8) Which relay is cycling when transitioning from menu to normal display right after boot?
// 9) Add a calibration menu item for baro
// 10) Add a menu item to display WiFi signal strength

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include <Filter.h>
#include <time.h>
#include <WiFi.h>
#include "LRThermostat.h"
#include "LRThermostat_menu.h"
#include "WifiCredentials.h"

#define DEBUG 1

// Hardware definitions
#define FAN_RELAY 16        // GPIO
#define HEAT_RELAY 17       // GPIO
#define AC_RELAY HEAT_RELAY // GPIO
#define DH_RELAY HEAT_RELAY // GPIO

// Misc defines
#define MENU_MAGIC_KEY 0xB00B
#define EEPROM_LOCAL_VAR_ADDR 0x100    // Leave the lower half for menu storage
#define INACTIVITY_TIMEOUT 10000       // 10000 mS = 10 sec
#define COMPRESSOR_DELAY (5 * 60)      // 5 minutes (counted in seconds)
#define LOOP_1_SEC 1000                // 1000 mS = 1 sec
#define T6_HOURS_IN_SEC (6 * 3600)     // 6 hours (counted in seconds)
#define T100_MS 100                    // 100 mS
#define UINT32_ERASED_VALUE 0xFFFFFFFF // erased value in eeprom/flash
#define A_KNOWN_GOOD_TIME 1600000000   // "9/13/2020 7:26:40 CST" in case you are wondering :)

// Relay control macros
#define FAN(a) digitalWrite(FAN_RELAY, ((a) ? (OFF) : (ON)))
#define HEAT(a) digitalWrite(HEAT_RELAY, ((a) ? (OFF) : (ON)))
#define COOL(a) digitalWrite(AC_RELAY, ((a) ? (OFF) : (ON)))
#define DH(a) digitalWrite(DH_RELAY, ((a) ? (OFF) : (ON)))

// Temp/humidity/pressure sensor - BME280
Adafruit_BME280 bme;

float curTemp = 0; // BME280
float curHumd = 0; // BME280
float curBaro = 0; // BME280

// The following variables are loaded from the menu
// START of tcMenu loaded variables
float tempCal = 0.0;       // calibration factor
float humdCal = 0.0;       // calibration factor
float baroCal = 0.0;       // calibration factor
float hysteresis;          //  (+/-) hysteresis/2 is centered around the set point (except for dh).
uint32_t dhMinRunTime = 0; // Dehumidifier minimim run time before shutting off

MODE mode, lastMode; // Current mode and last mode
FAN fan;             // fan state
// END of tcMenu loaded variables

bool ctlState = OFF; // heat/cool/dh state
bool fanState = OFF; // fan state
bool wait = FALSE;   // either compressor delay or min run time delay

bool menuChg = FALSE;       // tcMenu variable change flag (to signal save to EEPROm needed)
uint32_t lclVarChgTime = 0; // local variable change falg

uint32_t compressorDelay = COMPRESSOR_DELAY; // in cooling and dh modes, wait 5 min before restarting after last time
                                             // it was shut off. Same delay at boot up of thermostat.
uint32_t minRunTimeDelay = 0;                // Dehumidifier min runtime countdown
int16_t *pSetPt = NULL;                      // Pointer to current setpoint based upon mode

// The local variables that are backed up in EEPROM
EEPROM_LOCAL_VARS loc;            // local working variables
EEPROM_LOCAL_VARS chgdVars = {0}; // Changes to be committed

// Usage counters outside of the EEPROM 'loc' mirrored variables
uint32_t heatSeconds = 0;
uint32_t coolSeconds = 0;
uint32_t dhSeconds = 0;

// Function declarations
void checkLocalVarChanges();
void saveLocToEEPROM();
void LocalDisplayFunction(unsigned int encoderValue, RenderPressMode clicked);
void accumulateUsage();
void loadMenuChanges();
void readSensors();
void heatControl();
void acControl();
void dehumidifyControl();
void fanControl();
void myResetCallback();
void configEncoderForMode();
void takeOverDisplay();
void shutDownPrevMode();

// display func declarations
void dispMainTemp();
void dispTempSmall();
void dispTempSetPt();
void dispAcSetPt();
void dispCoolOn();
void dispCoolOff();
void dispCoolWait();
void dispHeatOn();
void dispHeatOff();
void dispDhOn();
void dispDhWait();
void dispDhOff();
void dispModeOff();
void dispFanOn();
void dispFanOff();
void dispHumiditySmall();
void dispHumidityBig();
void dispHumiditySetPt();
void dispBaro();

void timeSetup();
void wifiSetup();

// Main Arduino setup function
void setup()
{
    // Initialize the EEPROM class to 512 bytes of storage
    // 0x000-0x0FF  tcMenu
    // 0x100-0x1FF  local vars
    EEPROM.begin(0x200);

    // tcMenu
    setupMenu();

    // Fire up serial port
    Serial.begin(115200);

    // Menu timeout
    renderer.setResetIntervalTimeSeconds(10);

    // Set up the timeout callback
    renderer.setResetCallback(myResetCallback);

    // Additional hardware setup not done by tcMenu
    if (!bme.begin(0x76))
    {
        tft.println("BME280 error");
        while (1)
            ;
    }

    // Load initial menu values
    menuMgr.load(MENU_MAGIC_KEY);
    loadMenuChanges();

    // Read up nvm local variables
    // Should really check return code here...
    EEPROM.readBytes(EEPROM_LOCAL_VAR_ADDR, &loc, sizeof(EEPROM_LOCAL_VARS));
    Serial.printf("Menu+local data restored, 0x%04X\n", (uint32_t)EEPROM.readUShort(0x0));

    // Update boot related items in nvm. These changes will get pushed to EEPROM by
    // checkLocalVarChanges() in the main loop.
    loc.powerCycleCnt++;
    loc.bootTime = 0; // set to zero now, will update ASAP

    // Initialize the on time counters
    if (loc.heatSeconds == UINT32_ERASED_VALUE)
    {
        loc.heatSeconds = 0;
    }
    if (loc.coolSeconds == UINT32_ERASED_VALUE)
    {
        loc.coolSeconds = 0;
    }
    if (loc.dhSeconds == UINT32_ERASED_VALUE)
    {
        loc.dhSeconds = 0;
    }

    // Setup the rest of the hardware
    pinMode(FAN_RELAY, OUTPUT);
    FAN(OFF);
    pinMode(HEAT_RELAY, OUTPUT);
    HEAT(OFF);
    pinMode(AC_RELAY, OUTPUT);
    COOL(OFF);
    pinMode(DH_RELAY, OUTPUT);
    DH(OFF);

    // Wifi
    wifiSetup();

    // Get time
    timeSetup();
}

// Main Arduino control loop
void loop()
{
    uint32_t time1sec = millis() + LOOP_1_SEC;
    uint32_t time6hr = T6_HOURS_IN_SEC; // countdown
    uint32_t curTime;
    uint32_t wifiRetry = 0;
    bool wifiUp = FALSE;

    while (1)
    {
        curTime = millis();

        // Attempt Wifi connection, although not required to operate
        if (wifiUp == FALSE)
        {
            if (curTime >= wifiRetry)
            {
                if (WiFi.status() == WL_CONNECTED)
                {
                    wifiUp = TRUE;
                    Serial.println("wifi up -> " + WiFi.localIP().toString());
                    serverSetup();
                }
                else
                {
                    wifiRetry = curTime + T100_MS;
                }
            }
        }

        // Service tcMenu
        taskManager.runLoop();

        // Service 1 second loop timer
        if (time1sec <= curTime)
        {
            time1sec += LOOP_1_SEC;

            // Read sensor data
            readSensors();

            // Run control algorithm
            switch (mode)
            {
            case HEAT:
                heatControl();
                if (ctlState == ON)
                {
                    heatSeconds++;
                }
                break;

            case DEHUMIDIFY:
                dehumidifyControl();
                if (ctlState == ON)
                {
                    coolSeconds++;
                }
                break;

            case COOL:
                acControl();
                if (ctlState == ON)
                {
                    dhSeconds++;
                }
                break;

            case NO_MODE:
            default:
                // make sure control relays are off. Fan on is OK.
                HEAT(OFF);
                COOL(OFF);
                DH(OFF);
            }

            // handle fan relay
            fanControl();

            // decrement compressor delay to 0
            if (compressorDelay > 0)
            {
                compressorDelay--;
            }

            // decrement min run time to 0
            if (minRunTimeDelay > 0)
            {
                minRunTimeDelay--;
            }

            // Check for local var changes that need EEPROM update
            checkLocalVarChanges();

            if (loc.bootTime == 0)
            {
                time_t now;
                time(&now);

                // This may never happen if not connected to Wifi, but that should be OK
                if (now > A_KNOWN_GOOD_TIME) // "9/13/2020 12:26:40" in case you are wondering :)
                {
                    loc.bootTime = now;
                    Serial.printf("boot at %u\n", loc.bootTime);
                    Serial.printf("heat on %u\n", loc.heatSeconds);
                    Serial.printf("ac on   %u\n", loc.coolSeconds);
                    Serial.printf("dh on   %u\n", loc.dhSeconds);

                    if (loc.lastClear == UINT32_ERASED_VALUE)
                    {
                        loc.lastClear = 0;
                    }
                }
            }

            // Once every 6 hours, add in the control usage seconds.
            // This is delayed to minimize writes to the EEPROM.
            if (--time6hr == 0)
            {
                time6hr = T6_HOURS_IN_SEC;
                accumulateUsage();

                // the loc vars get committed to EPROM below
            }
        } // end of 1 sec loop

        // Save local vars if inactivity timeout
        if (lclVarChgTime && ((lclVarChgTime + INACTIVITY_TIMEOUT) < curTime))
        {
            saveLocToEEPROM();
        }
    }
}

// Compare local and eeprom vars. If local has changed, but has been constant
// for 10 seconds, update eeprom.
void checkLocalVarChanges()
{
    EEPROM_LOCAL_VARS eeprom;

    // Should really check return codes here...
    EEPROM.readBytes(EEPROM_LOCAL_VAR_ADDR, &eeprom, sizeof(EEPROM_LOCAL_VARS));

    // See if anything changed
    if (memcmp(&eeprom, &loc, sizeof(EEPROM_LOCAL_VARS)))
    {
        if (memcmp(&loc, &chgdVars, sizeof(EEPROM_LOCAL_VARS)))
        {
            // A change has been detected, save current state and time
            chgdVars = loc;
            lclVarChgTime = millis();
        }
    }
    else
    {
        lclVarChgTime = 0;
    }
}

// Save local variables to EEPROM
void saveLocToEEPROM()
{
    EEPROM.writeBytes(EEPROM_LOCAL_VAR_ADDR, &loc, sizeof(EEPROM_LOCAL_VARS));
    EEPROM.commit();
    Serial.printf("Local data saved\n");

    chgdVars = {0};
    lclVarChgTime = 0;
}

// Add non-saved usage data to 'loc' vars so they get saved to NVM
void accumulateUsage()
{
    loc.heatSeconds += heatSeconds;
    loc.coolSeconds += coolSeconds;
    loc.dhSeconds += dhSeconds;
    heatSeconds = 0;
    coolSeconds = 0;
    dhSeconds = 0;
}

// This function is called by the renderer every 100 mS once the display is taken over.
#define ENC_MAX 99
void localDisplayFunction(unsigned int encoderValue, RenderPressMode clicked)
{
    static bool myDispInit = FALSE;

#if 0
    if (pSetPt)
    {
        static int count = 10;
        count--;
        if (count == 0)
        {
            count = 10;
            Serial.printf("enc: %i, setp: %hu \n", encoderValue, *pSetPt);
        }
    }
#endif

    // No need to blacken the screen more than once.
    if (myDispInit == FALSE)
    {
        // Display init
        tft.fillScreen(TFT_BLACK);

        myDispInit = TRUE;
    }

    // Update setpoint if different; same encoder conversion here
    if (pSetPt)
    {
        *pSetPt = ENC_MAX - encoderValue;
    }

    // Items that get updated
    switch (mode)
    {
    case HEAT:
        dispMainTemp();
        dispHumiditySmall();
        dispTempSetPt();
        (ctlState == ON) ? dispHeatOn() : dispHeatOff();
        dispBaro();
        break;

    case COOL:
        dispMainTemp();
        dispHumiditySmall();
        dispAcSetPt();
        dispBaro();

        if (ctlState == OFF)
        {
            if (wait)
            {
                dispCoolWait();
            }
            else
            {
                dispCoolOff();
            }
        }
        else if (ctlState == ON)
        {
            dispCoolOn();
        }
        break;

    case DEHUMIDIFY:
        dispTempSmall();
        dispHumidityBig();
        dispHumiditySetPt();
        dispBaro();

        if (ctlState == OFF)
        {
            wait ? dispDhWait() : dispDhOff();
        }
        else if (ctlState == ON)
        {
            wait ? dispDhWait() : dispDhOn();
        }
        break;

    default:
    case NO_MODE:
        dispMainTemp();
        dispHumiditySmall();
        dispModeOff();
        dispBaro();

        break;
    }

    // fan state
    (fan == FAN_ON) ? dispFanOn() : dispFanOff();

    // Go to the menu when enter pressed
    if (clicked)
    {
        tft.setTextSize(1); // be sure to set back to default
        renderer.giveBackDisplay();
        myDispInit = FALSE;

        lastMode = mode;
    }
}

// Go fetch menu values from tcMenu
void loadMenuChanges()
{
    // First, save tcmenu values if changed
    if (menuChg)
    {
        // save menu to EEPROM
        menuMgr.save(MENU_MAGIC_KEY);
        EEPROM.commit();

        Serial.printf("Menu data saved, 0x%04X\n", (uint32_t)EEPROM.readUShort(0x0));
        menuChg = FALSE;
    }

    tempCal = menuTempCal.getLargeNumber()->getAsFloat();
    humdCal = menuHumidityCal.getLargeNumber()->getAsFloat();
    //    baroCal = menuPressureCal.getLargeNumber()->getAsFloat();
    baroCal = 1.44;
    mode = (MODE)menuModeEnum.getCurrentValue();
    fan = (FAN)menuFanEnum.getCurrentValue();
    dhMinRunTime = menuMinRunTime.getAsFloatingPointValue() * 60; // specified in min, convert to sec.

    switch (mode)
    {
    case HEAT:
        hysteresis = menuTempHysteresis.getLargeNumber()->getAsFloat() / 2;
        break;

    case COOL:
        hysteresis = menuCoolingHysteresis.getLargeNumber()->getAsFloat() / 2;
        break;

    case DEHUMIDIFY:
        //    hysteresis = menuHumdHysteresis.getLargeNumber()->getAsFloat() / 2;
        hysteresis = menuHumdHysteresis.getLargeNumber()->getAsFloat();
        break;

    default:
    case NO_MODE:
        break;
    }
}

// Read BME280 sensors. Use an exponential filter to filter out sensor noise.
// First, initialize the filter with an initial read. After that, each reading
// only allows approximately a 10% change in value, using the following formula:
// y(n) = ( w * x(n) ) + ( (1 – w) * y(n–1) )
// where 'w' is the weighting facor (ie 10%)
// So for example, if a normal temperature reading is 66.0 degrees, and then the
// next adc reading is 70.0 (which is a highly unlikely jump in temp), the filter
// will increase the reading by ~10%, or 0.4 deg instead of the full 4.0 deg.
// If you do experience a rapid change, the final reading will eventually be
// achieved after several iterations of the filter.
void readSensors()
{
    static bool readSensorsInit = FALSE;

    // Create exponential filters with a weight of 10%
    static ExponentialFilter<float> tempFilter(10, 0);
    static ExponentialFilter<float> humdFilter(10, 0);
    static ExponentialFilter<float> baroFilter(10, 0);

    if (readSensorsInit == FALSE)
    {
        // Throw out first reading
        bme.readTemperature();
        bme.readHumidity();
        bme.readPressure();

        // Read a second time to initialize the filter
        tempFilter.SetCurrent(bme.readTemperature());
        humdFilter.SetCurrent(bme.readHumidity());
        baroFilter.SetCurrent(bme.readPressure());
        readSensorsInit = TRUE;
    }

    // Run the filter
    tempFilter.Filter(bme.readTemperature());
    humdFilter.Filter(bme.readHumidity());
    baroFilter.Filter(bme.readPressure());

    // Extract and post-process the readings
    curTemp = tempFilter.Current() * 1.8 + 32 + tempCal;
    curHumd = humdFilter.Current() + humdCal;
    curBaro = baroFilter.Current() / 3386.39 + baroCal;
}

void heatControl()
{
    float set = loc.heatSetPt;

    if ((ctlState == ON) && (curTemp > (set + hysteresis)))
    {
        ctlState = OFF;
        HEAT(OFF);
    }
    else if ((ctlState == OFF) && (curTemp < (set - hysteresis)))
    {
        ctlState = ON;
        HEAT(ON);
    }
    else
    {
        // You're in-between, do nothing
    }

#if DEBUG
    static bool lastSt = OFF;
    static float lastSet = -1;

    if ((lastSt != ctlState) || (lastSet != set))
    {
        lastSt = ctlState;
        lastSet = set;

        Serial.printf("heat: cur:%0.2f  set:%0.2f  hys(+/-):%0.2f  %s\n",
                      curTemp, set, hysteresis, ctlState ? "ON" : "OFF");
    }
#endif
}

void acControl()
{
    float set = loc.acSetPt;

    if ((ctlState == ON) && (curTemp < (set - hysteresis)))
    {
        ctlState = OFF;
        COOL(OFF);
        compressorDelay = COMPRESSOR_DELAY;
    }
    else if ((ctlState == OFF) && (curTemp > (set + hysteresis)))
    {
        wait = (compressorDelay != 0);

        if (!wait)
        {
            ctlState = ON;
            COOL(ON);
        }
    }
    else
    {
        // You're in-between, do nothing
        wait = 0;
    }

#if DEBUG
    static bool lastSt = OFF;
    static float lastSet = -1;

    if ((lastSt != ctlState) || (lastSet != set) || compressorDelay)
    {
        lastSt = ctlState;
        lastSet = set;

        Serial.printf("cool: cur:%0.2f  set:%0.2f  hys(+/-):%0.2f  %s  cmpDly: %u\n",
                      curTemp, set, hysteresis, ctlState ? "ON" : "OFF", compressorDelay);
    }
#endif
}

void dehumidifyControl()
{
    float set = loc.dhSetPt;

    if ((ctlState == ON) && (curHumd <= (set /* - hysteresis */)))
    {
        wait = (minRunTimeDelay != 0);

        if (!wait)
        {
            ctlState = OFF;
            DH(OFF);
            compressorDelay = COMPRESSOR_DELAY;
        }
    }
    else if ((ctlState == OFF) && (curHumd >= (set + hysteresis)))
    {
        wait = (compressorDelay != 0);

        if (!wait)
        {
            ctlState = ON;
            DH(ON);
            minRunTimeDelay = dhMinRunTime;
        }
    }
    else
    {
        // You're in-between, do nothing
        wait = 0;
    }

#if DEBUG
    static bool lastSt = OFF;
    static float lastSet = -1;

    if ((lastSt != ctlState) || (lastSet != set) || compressorDelay || minRunTimeDelay)
    {
        lastSt = ctlState;
        lastSet = set;

        Serial.printf("dh: cur:%0.2f  set:%0.2f  hys(+N/-0):%0.2f  %s  cmpDly: %u  minRT: %u\n",
                      curHumd, set, hysteresis, ctlState ? "ON" : "OFF", compressorDelay, minRunTimeDelay);
    }
#endif
}

void fanControl()
{
    if ((fanState == ON) && (fan == FAN_AUTO))
    {
        FAN(OFF);
    }
    else if ((fanState == OFF) && (fan == FAN_ON))
    {
        FAN(ON);
    }
    else
    {
        // You're in-between, do nothing
    }
}

void configEncoderForMode()
{
    loadMenuChanges(); // We need to know if 'mode' has changed

    switch (mode)
    {
    case HEAT:
        pSetPt = &loc.heatSetPt;
        break;

    case COOL:
        pSetPt = &loc.acSetPt;
        break;

    case DEHUMIDIFY:
        pSetPt = &loc.dhSetPt;
        break;

    case NO_MODE:
        pSetPt = NULL;
    default:
        break;
    }

    // Encoder goes from 0 - 99 (ENC_MAX). Encoder is set to current set point
    switches.changeEncoderPrecision(ENC_MAX, pSetPt ? ENC_MAX - *pSetPt : 0);

    Serial.printf("Exit menu: Cur Mode: %hd, setpt: %hu\n", mode, pSetPt ? *pSetPt : 777);
}

void shutDownPrevMode()
{
    // Shut down previous mode if different
    if (lastMode != mode)
    {
        ctlState = OFF;

        // All relays off
        HEAT(OFF);
        COOL(OFF);
        DH(OFF);
    }
}

void takeOverDisplay()
{
    shutDownPrevMode();
    configEncoderForMode();
    renderer.takeOverDisplay(localDisplayFunction);
}

// This function is called when the menu becomes inactive.
void myResetCallback()
{
    takeOverDisplay();
}

// The tcMenu callbacks
void CALLBACK_FUNCTION ExitMenuCallback(int id)
{
    takeOverDisplay();
}

// The rest of these callbacks exist primarily to indicate that a menu
// item has been changed, so we know when to save to EEPROM.
void CALLBACK_FUNCTION ModeCallback(int id)
{
    int32_t val = menuModeEnum.getCurrentValue();
    Serial.printf("CB - Mode: %d\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION FanCallback(int id)
{
    int32_t val = menuFanEnum.getCurrentValue();
    Serial.printf("CB - Fan: %d\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION MinRunTimeCallback(int id)
{
    float val = menuMinRunTime.getAsFloatingPointValue();
    Serial.printf("CB - DhMinT: %0.1f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION HumidityCalCallback(int id)
{
    float val = menuHumidityCal.getLargeNumber()->getAsFloat();
    Serial.printf("CB - DhCal: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION HumdHysteresisCallback(int id)
{
    float val = menuHumdHysteresis.getLargeNumber()->getAsFloat();
    Serial.printf("CB - DhHys: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION TempCalCallback(int id)
{
    float val = menuTempCal.getLargeNumber()->getAsFloat();
    Serial.printf("CB - HeatCal: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION TempHysteresisCallback(int id)
{
    float val = menuTempHysteresis.getLargeNumber()->getAsFloat();
    Serial.printf("CB - HeatHys: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION CoolingCalCallback(int id)
{
    float val = menuCoolingCal.getLargeNumber()->getAsFloat();
    Serial.printf("CB - CoolCal: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION CoolingHysteresisCallback(int id)
{
    float val = menuCoolingHysteresis.getLargeNumber()->getAsFloat();
    Serial.printf("CB - CoolHys: %0.2f\n", val);
    menuChg = TRUE;
}

char *formatUsageCounter(uint32_t sec, char *buf)
{
    uint32_t hours = sec / 3600;
    sec -= hours * 3600;
    uint32_t min = sec / 60;
    sec -= min * 60;

    // output format: hhhh:mm:ss
    sprintf(buf, "%4u:%02u:%02u", hours, min, sec);

    return buf;
}

void CALLBACK_FUNCTION DisplayUsageCntrs(int id)
{
    accumulateUsage();
    saveLocToEEPROM();

    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0, 2);

    char buf[32];
    ctime_r((const time_t *)&loc.lastClear, buf);
    tft.printf("Accumulated time since:\n%s\n", &buf[4]);
    tft.printf("Heat on %s\n", formatUsageCounter(loc.heatSeconds, buf));
    tft.printf("A/C  on %s\n", formatUsageCounter(loc.coolSeconds, buf));
    tft.printf("D/H  on %s\n", formatUsageCounter(loc.dhSeconds, buf));
}

void CALLBACK_FUNCTION ClearUsageCntrs(int id)
{
    heatSeconds = 0;
    coolSeconds = 0;
    dhSeconds = 0;
    loc.heatSeconds = 0;
    loc.coolSeconds = 0;
    loc.dhSeconds = 0;

    time_t now;
    time(&now);

    if (now > A_KNOWN_GOOD_TIME)
    {
        loc.lastClear = now;
    }
    else
    {
        loc.lastClear = 0;
    }

    DisplayUsageCntrs(0);
}

void CALLBACK_FUNCTION SafePowerdown(int id)
{
    accumulateUsage();
    saveLocToEEPROM();

    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0, 2);

    tft.printf("It is safe to power off!\n");
}

//******************************** Display Routines *******************************************
void dispMainTemp()
{
    tft.setTextColor(TFT_CYAN, TFT_BLACK);     // Note: the new fonts do not draw the background colour
    tft.drawNumber(round(curTemp), 47, 42, 7); // Temperature using font 7
    tft.drawString("O", 114, 37, 2);
}

//****************************************** Temperature Set Point ****************************
void dispTempSetPt()
{
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("SET", 100, 8, 1);
    tft.setTextSize(2);
    if (loc.heatSetPt > 9)
    {
        tft.drawNumber((uint32_t)loc.heatSetPt, 123, 8, 1); // font 1
    }
    else
    {
        tft.drawString(" ", 123, 8, 1);
        tft.drawNumber((uint32_t)loc.heatSetPt, 135, 8, 1); // font 1
    }
    tft.setTextSize(1);
    tft.drawString(" o", 145, 6, 1);
}

void dispAcSetPt()
{
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("SET", 100, 8, 1);
    tft.setTextSize(2);
    if (loc.acSetPt > 9)
    {
        tft.drawNumber((uint32_t)loc.acSetPt, 123, 8, 1); // font 1
    }
    else
    {
        tft.drawString(" ", 123, 8, 1);
        tft.drawNumber((uint32_t)loc.acSetPt, 135, 8, 1); // font 1
    }
    tft.setTextSize(1);
    tft.drawString(" o", 145, 6, 1);
}

// ********************** Humidity Reading for when temperature is main screen ****************
void dispHumiditySmall()
{
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawNumber(round(curHumd), 8, 8, 1); // Humidity at font 1
    tft.setTextSize(1);
    tft.drawString(" %", 30, 6, 1);
}

//****************************************** MODES ********************************************
void dispCoolOff()
{
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("COOL    ", 8, 115, 1);
    tft.drawString("OFF ", 8, 105, 1);
}

void dispCoolWait()
{
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("COOL    ", 8, 115, 1);
    tft.drawString("WAIT", 8, 105, 1);
}

void dispCoolOn()
{
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.drawString("COOL    ", 8, 115, 1);
    tft.drawString("ON  ", 8, 105, 1);
}

void dispHeatOff()
{
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("HEAT    ", 8, 115, 1);
    tft.drawString("OFF", 8, 105, 1);
}

void dispHeatOn()
{
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("HEAT    ", 8, 115, 1);
    tft.drawString("ON ", 8, 105, 1);
}

void dispDhOff()
{
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("DEHUMIDIFY", 8, 115, 1);
    tft.drawString("OFF ", 8, 105, 1);
}

void dispDhWait()
{
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("DEHUMIDIFY", 8, 115, 1);
    tft.drawString("WAIT", 8, 105, 1);
}

void dispDhOn()
{
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.drawString("DEHUMIDIFY", 8, 115, 1);
    tft.drawString("ON  ", 8, 105, 1);
}

void dispModeOff()
{
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("          ", 8, 105, 1);
    tft.drawString("OFF       ", 8, 115, 1);
}

//****************************************** Baro *********************************************
void dispBaro()
{
    tft.setTextColor(TFT_GOLD, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawFloat(curBaro, 1, 108, 108, 1); // drawFloat does appropriate rounding
    tft.setTextSize(1);
    tft.drawString("inHg", 131, 94, 1);
}

//****************************************** Fan **********************************************
void dispFanOn()
{
    tft.setTextColor(TFT_VIOLET, TFT_BLACK);
    tft.drawString("FAN    ", 8, 57, 1);
    tft.drawString("ON  ", 8, 67, 1);
}

void dispFanOff()
{
    tft.setTextColor(TFT_VIOLET, TFT_BLACK);
    tft.drawString("       ", 8, 57, 1);
    tft.drawString("    ", 8, 67, 1);
}

// ************************* Humidity Reading for when Humidity is main screen ****************
void dispHumidityBig()
{
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawNumber(round(curHumd), 47, 42, 7); // Temperature using font 7

    tft.setTextSize(2);
    tft.drawString(" %", 103, 40, 1);
    tft.setTextSize(1);
}

// ****************************** Humidity set point ******************************************
void dispHumiditySetPt()
{
    tft.setTextColor(TFT_PINK, TFT_BLACK);
    tft.setTextSize(2);
    if (loc.dhSetPt > 9)
    {
        tft.drawNumber((uint32_t)loc.dhSetPt, 8, 8, 1); // font 1
    }
    else
    {
        tft.drawString(" ", 8, 8, 1);
        tft.drawNumber((uint32_t)loc.dhSetPt, 20, 8, 1); // font 1
    }
    tft.setTextSize(1);
    tft.drawString(" %", 30, 6, 1);
    tft.drawString("SET", 48, 8, 1);
}

//******************************** Temperature in Dehumidity Mode *****************************
void dispTempSmall()
{
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawNumber(round(curTemp), 123, 8, 1); // font 1
    tft.setTextSize(1);
    tft.drawString(" o", 145, 6, 1);
}

//*********************************************************************************************
const char *timeZone = "CST6CDT,M3.2.0,M11.1.0";
const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";

void timeSetup()
{
    // init and get the time
    configTzTime(timeZone, ntpServer1, ntpServer2);
}

//*********************************************************************************************
void wifiSetup()
{
    Serial.printf("Connecting to: %s\n", ssid);
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
}
