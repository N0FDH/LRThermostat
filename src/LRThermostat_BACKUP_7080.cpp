// Copyright © 2022 Randy Rysavy <randy.rysavy@gmail.com>
// This work is free. You can redistribute it and/or modify it under the
// terms of the Do What The Fuck You Want To Public License, Version 2,
// as published by Sam Hocevar. See http://www.wtfpl.net/ for more details.

// Features to add / bugs to fix
// 1) on time counters for all modes
// 2) An 'eject' like menu item that saves NVM stuff before power down
// 3) A way to clear the mode time counters

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include <Filter.h>
#include <time.h>
#include "LRThermostat.h"
#include "LRThermostat_menu.h"

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
#define COMPRESSOR_DELAY 45            // TODO: (5 * 60)   // 5 minutes (counted in seconds)
#define LOOP_1_SEC 1000                // 1000 mS = 1 sec
#define T24_HOURS_IN_SEC 60            // (3600 * 24)
#define UINT32_ERASED_VALUE 0xFFFFFFFF // erased value in eeprom/flash

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
float hysteresis;          //  (+/-) hysteresis/2 is centered around the set point.
uint32_t dhMinRunTime = 0; // Dehumidifier minimim run time before shutting off

<<<<<<< HEAD
MODE mode, lastMode; // Current mode and last mode
FAN fan;             // fan state
=======
// Note: tcMenu does not provide enums like this. Be sure to update these
// if/when you change the 'mode' or 'fan' variable in tcMenu. Orders must match!
typedef enum
{
    NO_MODE,   // 0
    HEAT,      // 1
    COOL,      // 2
    DEHUMIDIFY // 3
} MODE;
MODE mode, lastMode;

typedef enum
{
    FAN_ON,
    FAN_AUTO
} FAN;
FAN fan;
>>>>>>> df3478670106d3b73decef513c7a6b1853f10fe0
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

// Function declarations
void checkLocalVarChanges();
void LocalDisplayFunction(unsigned int encoderValue, RenderPressMode clicked);
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

void setupTime();
void printLocalTime();

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

    // Wifi
    wifiSetup();

    // Get time
    setupTime();
}

// Main Arduino control loop
void loop()
{
    uint32_t time1sec = 0;
    uint32_t time24hr = T24_HOURS_IN_SEC; // countdown
    uint32_t curTime;
    uint32_t heatSeconds = 0;
    uint32_t coolSeconds = 0;
    uint32_t dhSeconds = 0;

    while (1)
    {
        // tcMenu
        taskManager.runLoop();

        curTime = millis();

        // 1 second loop time
        if (time1sec <= curTime)
        {
            time1sec = curTime + LOOP_1_SEC;

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

                if (now > 1600000000) // 9/13/2020 12:26:40 in case you are wondering :)
                {
                    loc.bootTime = now;
                    Serial.printf("boot at %u\n", loc.bootTime);
                    Serial.printf("heat on %u\n", loc.heatSeconds);
                    Serial.printf("ac on   %u\n", loc.coolSeconds);
                    Serial.printf("dh on   %u\n", loc.dhSeconds);
                }
            }

            // Once every 24 hours, add in the control usage seconds.
            // This is to minimize writes to the EEPROM.
            if (--time24hr == 0)
            {
                time24hr = T24_HOURS_IN_SEC;

                Serial.printf("heat on %u, %u\n", heatSeconds, loc.heatSeconds);
                Serial.printf("ac on   %u, %u\n", coolSeconds, loc.coolSeconds);
                Serial.printf("dh on   %u, %u\n", dhSeconds, loc.dhSeconds);
                loc.heatSeconds += heatSeconds;
                loc.coolSeconds += coolSeconds;
                loc.dhSeconds += dhSeconds;
                heatSeconds = 0;
                coolSeconds = 0;
                dhSeconds = 0;

                // the loc vars get committed to EPROM above
            }
        } // end of 1 sec loop

        // Save local vars if inactivity timeout
        if (lclVarChgTime && ((lclVarChgTime + INACTIVITY_TIMEOUT) < curTime))
        {
            EEPROM.writeBytes(EEPROM_LOCAL_VAR_ADDR, &loc, sizeof(EEPROM_LOCAL_VARS));
            EEPROM.commit();
            Serial.printf("Local data saved\n");

            chgdVars = {0};
            lclVarChgTime = 0;
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
        break;

    case COOL:
        dispMainTemp();
        dispHumiditySmall();
        dispAcSetPt();

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
        hysteresis = menuHumdHysteresis.getLargeNumber()->getAsFloat() / 2;
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

    if ((ctlState == ON) && (curHumd < (set - hysteresis)))
    {
        wait = (minRunTimeDelay != 0);

        if (!wait)
        {
            ctlState = OFF;
            DH(OFF);
            compressorDelay = COMPRESSOR_DELAY;
        }
    }
    else if ((ctlState == OFF) && (curHumd > (set + hysteresis)))
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

        Serial.printf("dh: cur:%0.2f  set:%0.2f  hys(+/-):%0.2f  %s  cmpDly: %u  minRT: %u\n",
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

//******************************** Display Routines *******************************************
void dispMainTemp()
{
    tft.setTextColor(TFT_CYAN, TFT_BLACK);     // Note: the new fonts do not draw the background colour
    tft.drawNumber(round(curTemp), 43, 45, 7); // Temperature using font 7
    tft.drawString("O", 110, 35, 2);
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

//****************************************** Fan **********************************************
void dispFanOn()
{
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("FAN    ", 135, 115, 1);
    tft.drawString("ON  ", 135, 105, 1);
}

void dispFanOff()
{
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("       ", 135, 115, 1);
    tft.drawString("    ", 135, 105, 1);
}

// ************************* Humidity Reading for when Humidity is main screen ****************
void dispHumidityBig()
{
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawNumber(round(curHumd), 43, 45, 7);
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

void setupTime()
{
    // init and get the time
    configTzTime(timeZone, ntpServer1, ntpServer2);
}
