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
// 7) DONE - Which relay is cycling when transitioning from menu to normal display right after boot?
// 8) DONE - Add a calibration menu item for baro // 8.1) Remove baro rapid tcmenu item
// 9) !TODO - Write main HTML status page including uptime and "on times"
// 10) !TODO - Add a menu item to display:
//                WiFi signal strength
//                SSID
//                IP addr
//                MAC
//                FW version
//                Uptime
// 11) DONE - Change all display routines to x,y variable based.
// 12) DONE - Clean up DH main display screen (swap set & temp, see whats up with main hum%)

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include <Filter.h>
#include <time.h>
#include <WiFi.h>
#include <Esp.h>
#include "LRThermostat.h"
#include "LRThermostat_menu.h"
#include "WifiCredentials.h"

#define VERSION "1.1"
#define LORENS_PREFERENCES 1

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
#define T_10MIN_IN_SEC (10 * 60)       // 10 min (counted in seconds)
#define T_6HOURS_IN_SEC (6 * 3600)     // 6 hours (counted in seconds)
#define T_100MS 100                    // 100 mS
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

#define BARO_FLOAT_TO_INT(a) ((int32_t)(((a) + 0.0005) * 1000)) // keep integer + 3 decimal places, rounded first
int32_t baroDir = 0; // 0 == steady, (+/-)1 == rise/fall, (+/-)2 == rapid rise/fall

// The following variables are loaded from the menu
// START of tcMenu loaded variables
float tempCal = 0.0;       // calibration factor
float humdCal = 0.0;       // calibration factor
float baroCal = 0.0;       // calibration factor
float hysteresis;          //  (+/-) hysteresis/2 is centered around the set point (except for dh).
uint32_t dhMinRunTime = 0; // Dehumidifier minimim run time before shutting off

MODE mode, lastMode; // Current mode and last mode
FAN fan;             // fan state

int32_t baroSteady; // upper baro pressure limit for 'steady'
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
float readTemperature();
float readHumidity();
float readPressure();
void initBME280();
void checkLocalVarChanges();
void saveLocToEEPROM();
void saveMenuToEEPROM();
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
void shutDownPrevMode(bool force);
void updateBaroRiseFall();

// display func declarations
void dispMain();
void dispSmall();
void dispSetPt();
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
void dispBaro();

void timeSetup();
void wifiSetup();

void graphit();

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
    initBME280();

    // Load initial menu values
    menuMgr.load(MENU_MAGIC_KEY);
    loadMenuChanges();

    // pre-fill lastMode on startup
    lastMode = mode;

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

    // Temp hack into the graphing function
    //    graphit();
}

// Main Arduino control loop
void loop()
{
    uint32_t time1sec = millis() + LOOP_1_SEC;
    uint32_t time10min = 3;             // countdown
    uint32_t time6hr = T_6HOURS_IN_SEC; // countdown
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
                    wifiRetry = curTime + T_100MS;
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
                if (now > A_KNOWN_GOOD_TIME)
                {
                    loc.bootTime = now;
                    Serial.printf("boot at %u\n", loc.bootTime);

                    if (loc.lastClear == UINT32_ERASED_VALUE)
                    {
                        loc.lastClear = 0;
                    }
                }
            }

            // Once every 10 min, check on barometric pressure and adjust
            // rising, falling indicator.
            if (--time10min == 0)
            {
                updateBaroRiseFall();
                time10min = T_10MIN_IN_SEC;
            }

            // Once every 6 hours, add in the control usage seconds.
            // This is delayed to minimize writes to the EEPROM.
            if (--time6hr == 0)
            {
                time6hr = T_6HOURS_IN_SEC;
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

// Save tcMenu variables to EEPROM
void saveMenuToEEPROM()
{
    // save menu to EEPROM
    menuMgr.save(MENU_MAGIC_KEY);
    EEPROM.commit();

    Serial.printf("Menu data saved, 0x%04X\n", (uint32_t)EEPROM.readUShort(0x0));
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

#define BARO_CNT 18
#define BARO_IDX_10MIN 0
#define BARO_IDX_20MIN 1
#define BARO_IDX_3HR (BARO_CNT - 1)
void updateBaroRiseFall()
{
    static int16_t oldBaro[BARO_CNT] = {0}; // 3 hrs history, once every 10 min

    // Get baro int
    int32_t curBaroInt = BARO_FLOAT_TO_INT(curBaro);

    // determine if ever initialized
    if (oldBaro[0] == 0)
    {
        for (int i = 0; i < BARO_CNT; i++)
        {
            oldBaro[i] = curBaroInt;
        }
    }

    // See how much we have shifted in the 3 intervals
    int32_t diff10min = curBaroInt - oldBaro[BARO_IDX_10MIN];
    int32_t diff20min = curBaroInt - oldBaro[BARO_IDX_20MIN];
    int32_t diff3hr = curBaroInt - oldBaro[BARO_IDX_3HR];

    // https://www.faa.gov/documentLibrary/media/Order/JO_7900.5E_with_Change_1.pdf, p.76
    // g.Pressure Falling Rapidly. Pressure falling rapidly occurs when station pressure falls at
    // the rate of at least .06 inch(2.03 hPa) or more per hour which totals 0.02 inch(0.68 hPa)
    // or more at time of observation. (SAME FOR RISING).

    // Check for rapid change
    if ((((diff10min > 0) && (diff20min > 0)) ||
         ((diff10min < 0) && (diff20min < 0))) && // both either + or -
        ((abs(diff10min) >= 10) &&                // rate >= .06 in/hr (.060 in/hr == 0.010 in/10min)
         (abs(diff20min) >= 20)))                 // total >= .02 in at time of observation
    {
        baroDir = (diff10min > 0) ? 2 : -2;
    }
    // Check for steady
    else if (abs(diff3hr) <= baroSteady)
    {
        baroDir = 0;
    }
    // Everything else is a slow or moderate change
    else
    {
        baroDir = (diff3hr > 0) ? 1 : -1;
    }

    Serial.printf("curBaro: %i, b10m: %i/%hi, b20m: %i/%hi, b3hr: %i/%hi, steady: %i, dir: %i\n",
                  curBaroInt,
                  oldBaro[BARO_IDX_10MIN], diff10min,
                  oldBaro[BARO_IDX_20MIN], diff20min,
                  oldBaro[BARO_IDX_3HR], diff3hr,
                  baroSteady, baroDir);

    // shift baro history down and capture most recent
    for (int i = (BARO_CNT - 1); i > 0; i--)
    {
        oldBaro[i] = oldBaro[i - 1];
    }
    oldBaro[0] = curBaroInt;
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

    dispMain();
    dispSmall();
    dispBaro();
    dispSetPt();

    // Items that get updated
    switch (mode)
    {
    case HEAT:
        (ctlState == ON) ? dispHeatOn() : dispHeatOff();
        break;

    case COOL:
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
        dispModeOff();

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
        saveMenuToEEPROM();
        menuChg = FALSE;
    }

    tempCal = menuTemperatureCal.getLargeNumber()->getAsFloat();
    humdCal = menuHumidityCal.getLargeNumber()->getAsFloat();
    baroCal = menuPressureCal.getLargeNumber()->getAsFloat();

    mode = (MODE)menuModeEnum.getCurrentValue();
    fan = (FAN)menuFanEnum.getCurrentValue();
    dhMinRunTime = menuMinRunTime.getAsFloatingPointValue() * 60; // specified in min, convert to sec.

    // First an initialization step. THis is working around a bug in tcMenu. Too long
    // and uninteresting to explain.
    if (menuBaroSteadyUpLimit.getLargeNumber()->isNegative())
    {
        menuBaroSteadyUpLimit.getLargeNumber()->setNegative(FALSE);
    }

    // Get baro limit as an integer
    baroSteady = BARO_FLOAT_TO_INT(menuBaroSteadyUpLimit.getLargeNumber()->getAsFloat());

    switch (mode)
    {
    case HEAT:
        hysteresis = menuHeatingHysteresis.getLargeNumber()->getAsFloat() / 2;
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
        readTemperature();
        readHumidity();
        readPressure();

        // Read a second time to initialize the filter
        tempFilter.SetCurrent(readTemperature());
        humdFilter.SetCurrent(readHumidity());
        baroFilter.SetCurrent(readPressure());
        readSensorsInit = TRUE;
    }

    // Run the filter
    tempFilter.Filter(readTemperature());
    humdFilter.Filter(readHumidity());
    baroFilter.Filter(readPressure());

    // Extract and post-process the readings
    curTemp = (tempFilter.Current() * 1.8) + 32 + tempCal;
    curHumd = humdFilter.Current() + humdCal;
    curBaro = (baroFilter.Current() / 3386.39) + baroCal;
}

void initBME280()
{
    if (!bme.begin(0x76))
    {
        // We're screwed
        tft.println("BME280 error");
        while (1)
            ;
    }
    Serial.printf("BME280 initialized\n");
}

float readPressure()
{
    float v;
    while ((v = bme.readPressure()) == NAN)
    {
        initBME280();
    }
    return v;
}

float readHumidity()
{
    float v;
    while ((v = bme.readHumidity()) == NAN)
    {
        initBME280();
    }
    return v;
}

float readTemperature()
{
    float v;
    while ((v = bme.readTemperature()) == NAN)
    {
        initBME280();
    }
    return v;
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

void shutDownPrevMode(bool force)
{
    // Shut down previous mode if different
    if (force || (lastMode != mode))
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
    shutDownPrevMode(FALSE);
    configEncoderForMode();
    renderer.takeOverDisplay(localDisplayFunction);
}

// This function is called when the menu becomes inactive.
void myResetCallback()
{
    takeOverDisplay();
}

// The tcMenu callbacks
void CALLBACK_FUNCTION ExitCallback(int id)
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
    float val = menuTemperatureCal.getLargeNumber()->getAsFloat();
    Serial.printf("CB - HeatCal: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION HeatingHysteresisCallback(int id)
{
    float val = menuHeatingHysteresis.getLargeNumber()->getAsFloat();
    Serial.printf("CB - HeatHys: %0.2f\n", val);
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
    // Save all variables to NVM
    accumulateUsage();
    saveLocToEEPROM();
    saveMenuToEEPROM();

    // Shut down thermostat functions
    shutDownPrevMode(TRUE);

    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0, 2);

    tft.printf("It is safe to power off!\n\n");
    tft.printf("The system will restart\nin 60 seconds.\n");

    // The ONLY delay() found in this FW :)
    delay(60 * 1000);
    ESP.restart();
}

void CALLBACK_FUNCTION BaroSteadyUpLimitCallback(int id)
{
    float val = menuBaroSteadyUpLimit.getLargeNumber()->getAsFloat();
    Serial.printf("CB - BaroSteady: %0.4f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION BaroRapidLoLimitCallback(int id)
{
    Serial.printf("TODO: REMOVE!\n");
}

void CALLBACK_FUNCTION PressureCalCallback(int id)
{
    float val = menuPressureCal.getLargeNumber()->getAsFloat();
    Serial.printf("CB - PressCal: %0.2f\n", val);
    menuChg = TRUE;
}

//******************************** Display Routines *******************************************
void dispMain()
{
    int32_t x = 47;
    int32_t y = 42;

    if (mode == DEHUMIDIFY)
    {
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        tft.drawNumber(round(curHumd), x, y, 7); // Temperature using font 7
        tft.drawString("%", x + 67, y - 5, 2);
    }
    else
    {
        tft.setTextColor(TFT_CYAN, TFT_BLACK);   // Note: the new fonts do not draw the background colour
        tft.drawNumber(round(curTemp), x, y, 7); // Temperature using font 7
        tft.drawString("O", x + 67, y - 5, 2);
    }
}

//******************************** Secondary Display ******************************************
void dispSmall()
{
    int32_t x = 8;
    int32_t y = 8;

    if (mode == DEHUMIDIFY)
    {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setTextSize(2);
        tft.drawNumber(round(curTemp), x, y, 1); // font 1
        tft.setTextSize(1);
        tft.drawString(" o", x + 22, y - 2, 1);
    }
    else
    {
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        tft.setTextSize(2);
        tft.drawNumber(round(curHumd), x, y, 1); // Humidity at font 1
        tft.setTextSize(1);
        tft.drawString(" %", x + 22, y - 2, 1);
    }
}

//******************************* Set point ***************************************************
void dispSetPt()
{
    int32_t x = 100;
    int32_t y = 8;

    if (mode == NO_MODE)
    {
        return;
    }

    tft.setTextColor(mode == DEHUMIDIFY ? TFT_PINK : TFT_YELLOW, TFT_BLACK);
    tft.drawString("SET", x, y, 1);
    tft.setTextSize(2);
    if (*pSetPt > 9)
    {
        tft.drawNumber((uint32_t)(*pSetPt), x + 23, y, 1); // font 1
    }
    else
    {
        tft.drawString(" ", x + 23, y, 1);
        tft.drawNumber((uint32_t)(*pSetPt), x + 35, y, 1); // font 1
    }
    tft.setTextSize(1);
    tft.drawString(mode == DEHUMIDIFY ? " %" : " o", x + 45, y - 2, 1);
}

//****************************************** MODES ********************************************
#define MODE_DISP_X 8
#define MODE_DISP_Y 105
void dispCoolOff()
{
    int32_t x = MODE_DISP_X;
    int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("OFF ", x, y, 1);
    tft.drawString("COOL    ", x, y + 10, 1);
}

void dispCoolWait()
{
    int32_t x = MODE_DISP_X;
    int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("WAIT", x, y, 1);
    tft.drawString("COOL    ", x, y + 10, 1);
}

void dispCoolOn()
{
    int32_t x = MODE_DISP_X;
    int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.drawString("ON  ", x, y, 1);
    tft.drawString("COOL    ", x, y + 10, 1);
}

void dispHeatOff()
{
    int32_t x = MODE_DISP_X;
    int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("OFF", x, y, 1);
    tft.drawString("HEAT    ", x, y + 10, 1);
}

void dispHeatOn()
{
    int32_t x = MODE_DISP_X;
    int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("ON ", x, y, 1);
    tft.drawString("HEAT    ", x, y + 10, 1);
}

void dispDhOff()
{
    int32_t x = MODE_DISP_X;
    int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("OFF ", x, y, 1);
    tft.drawString("DEHUMIDIFY", x, y + 10, 1);
}

void dispDhWait()
{
    int32_t x = MODE_DISP_X;
    int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("WAIT", x, y, 1);
    tft.drawString("DEHUMIDIFY", x, y + 10, 1);
}

void dispDhOn()
{
    int32_t x = MODE_DISP_X;
    int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.drawString("ON  ", x, y, 1);
    tft.drawString("DEHUMIDIFY", x, y + 10, 1);
}

void dispModeOff()
{
    int32_t x = MODE_DISP_X;
    int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("          ", x, y, 1);
    tft.drawString("OFF       ", x, y + 10, 1);
}

//****************************************** Fan **********************************************
#define FAN_DISP_X 8
#define FAN_DISP_Y 57
void dispFanOn()
{
    int32_t x = FAN_DISP_X;
    int32_t y = FAN_DISP_Y;

    tft.setTextColor(TFT_VIOLET, TFT_BLACK);
    tft.drawString("FAN    ", x, y, 1);
    tft.drawString("ON  ", x, y + 10, 1);
}

void dispFanOff()
{
    int32_t x = FAN_DISP_X;
    int32_t y = FAN_DISP_Y;

    tft.setTextColor(TFT_VIOLET, TFT_BLACK);
    tft.drawString("       ", x, y, 1);
    tft.drawString("    ", x, y + 10, 1);
}

//****************************************** Baro *********************************************
void dispBaro()
{
#if LORENS_PREFERENCES
    int32_t digits = 1;
    int32_t x = 97;
#else
    int32_t digits = 2;
    int32_t x = 85;
#endif
    int32_t y = 108;

    // Display the directional arrow
    int16_t color = (abs(baroDir) > 1) ? TFT_RED : TFT_GOLD;

    int32_t up = TFT_BLACK;
    int32_t dn = TFT_BLACK;
    int32_t ln = TFT_BLACK;

    if (baroDir < 0)
    {
        dn = color;
        ln = color;
    }
    else if (baroDir > 0)
    {
        up = color;
        ln = color;
    }

    tft.drawLine(x + 1, y + 3, x + 4, y + 0, up);
    tft.drawLine(x + 4, y + 0, x + 7, y + 3, up);

    tft.drawLine(x + 1, y + 10, x + 4, y + 13, dn);
    tft.drawLine(x + 4, y + 13, x + 7, y + 10, dn);

    tft.drawLine(x + 4, y + 0, x + 4, y + 13, ln);

    // Display numeric reading
    tft.setTextColor(TFT_GOLD, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawFloat(curBaro, digits, x + 11, y, 1); // drawFloat does appropriate rounding
    tft.setTextSize(1);
    tft.drawString("inHg", digits == 1 ? x + 34 : x + 46, y - 13, 1);
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
