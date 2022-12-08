// Copyright © 2022 Randy Rysavy <randy.rysavy@gmail.com>
// This work is free. You can redistribute it and/or modify it under the
// terms of the Do What The Fuck You Want To Public License, Version 2,
// as published by Sam Hocevar. See http://www.wtfpl.net/ for more details.

// Features to add / bugs to fix
// - Write main HTML status page including uptime and "on times"
// - Sync to ntp if possible (why?)
// - setpoint line on graphs should be conditional to MODE
// - add a "this cycle run time" counter and add to display
//   * keep on display after cycle finishes too
// - log on and off times (maybe last 25 ?)
//   - add ability to display last 8 on display
//   - do some statistics too -- avg run time, run count last 24 hrs
//   - Change WiFi monitoring to event driven
// DONE - add a total cycle count alongside the total runtime
// DONE - change to 24 save to EEPROM instead of 6 hrs. No save if OFF.

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include <Filter.h>
#include <time.h>
#include <WiFi.h>
#include <Esp.h>
#include <ESPmDNS.h>
#include <CircularBuffer.h>
#include "LRThermostat.h"
#include "LRThermostat_menu.h"
#include "WifiCredentials.h"
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

#define DEBUG 1

// EEPROM map:
// 0x000-0x0FF: tcMenu (256 bytes)
// 0x100-0x1FF: local vars (256 bytes)
// 0x200-0x3FF: credentials (512 bytes)

// Misc defines
#define MENU_MAGIC_KEY 0xB00B
#define LOC_MAGIC_KEY 0xB16B00B5
#define EEPROM_LOCAL_VAR_ADDR 0x100    // Leave the lower half for menu storage
#define EEPROM_CREDENTIALS_ADDR 0x200  // This leaves 256 bytes for local vars
#define INACTIVITY_TIMEOUT 10000       // 10000 mS = 10 sec
#define COMPRESSOR_DELAY (5 * 60)      // 5 minutes (counted in seconds)
#define LOOP_1_SEC 1000                // 1000 mS = 1 sec
#define T_10MIN_IN_SEC (10 * 60)       // 10 min (counted in seconds)
#define T_1MIN_IN_SEC (1 * 60)         // 1 min (counted in seconds)
#define T_24HOURS_IN_SEC (24 * 3600)   // 24 hours (counted in seconds)
#define T_100MS 100                    // 100 mS
#define UINT32_ERASED_VALUE 0xFFFFFFFF // erased value in eeprom/flash
#define A_KNOWN_GOOD_TIME 1600000000   // "9/13/2020 7:26:40 CST" in case you are wondering :)

// Relay control macros
#if (PCB_VERSION == 0)
#define FAN(a) digitalWrite(FAN_RELAY, ((a) ? (OFF) : (ON)))
#define HEAT(a) digitalWrite(HEAT_RELAY, ((a) ? (OFF) : (ON)))
#define COOL(a) digitalWrite(AC_RELAY, ((a) ? (OFF) : (ON)))
#define DH(a) digitalWrite(DH_RELAY, ((a) ? (OFF) : (ON)))
#else
#define FAN(a) digitalWrite(FAN_RELAY, ((a) ? (ON) : (OFF)))
#define HEAT(a) digitalWrite(HEAT_RELAY, ((a) ? (ON) : (OFF)))
#define COOL(a) digitalWrite(AC_RELAY, ((a) ? (ON) : (OFF)))
#define DH(a) digitalWrite(DH_RELAY, ((a) ? (ON) : (OFF)))
#endif

#define TOGGLE_LED() digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED))

// **************************** InfluxDB *****************************
// InfluxDB client instance
InfluxDBClient influxdb;
bool influxdbUp = FALSE;

// Data point
// items to capture:
// - humidity
// - temp
// - baro
// - mode
// - state
Point lrtData("LRT");

// Temp/humidity/pressure sensor - BME280
Adafruit_BME280 bme;

float curTemp = 0; // BME280
float curHumd = 0; // BME280
float curBaro = 0; // BME280

//#define BARO_FLOAT_TO_INT(a) ((int32_t)(((a) + 0.0005) * 1000)) // keep integer + 3 decimal places
#define BARO_FLOAT_TO_INT(a) ((int32_t)(round((a)*1000))) // keep integer + 3 decimal places
#define TEMP_FLOAT_TO_INT(a) ((int32_t)(round((a)*100)))  // keep integer + 2 decimal places
#define HUMD_FLOAT_TO_INT(a) TEMP_FLOAT_TO_INT(a)

int32_t baroDir = 0; // 0 == steady, (+/-)1 == rise/fall, (+/-)2 == rapid rise/fall
CircularBuffer<int16_t, HIST_CNT> cbBaro;
CircularBuffer<int16_t, HIST_CNT> cbHumd;
CircularBuffer<int16_t, HIST_CNT> cbTemp;

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
char *pSsid = NULL;                          // Pointer to booted SSID

// The local variables that are backed up in EEPROM
EEPROM_LOCAL_VARS loc;            // local working variables
EEPROM_LOCAL_VARS chgdVars = {0}; // Changes to be committed

// The local credentials that are backed up in EEPROM
EEPROM_CREDENTIALS creds; // credential working variables

// Usage counters outside of the EEPROM 'loc' mirrored variables
uint32_t heatSeconds = 0;
uint32_t coolSeconds = 0;
uint32_t dhSeconds = 0;
uint32_t heatCount = 0;
uint32_t coolCount = 0;
uint32_t dhCount = 0;

// setting PWM properties
const uint32_t pwmFreq = 5000;
const uint32_t pwmChannel = 0;
const uint32_t pwmResolution = 8;
// This a lot of trial and error, but comes out pretty good 10-100% in 10% increments
const uint8_t pwmDutyCycle[] = {1, 3, 7, 13, 23, 40, 69, 120, 185, 255};
#define PWM_MAX (sizeof(pwmDutyCycle) / sizeof(pwmDutyCycle[0]) - 1)
int32_t pwmCount = PWM_MAX;
bool pwmDirUp = false;
#define PWM_TEST 0

// Function declarations
void initBme280();
void readBme280(float_t *pP, float_t *pT, float_t *pH);
void checkLocalVarChanges();
void saveLocToEEPROM();
void saveCredsToEEPROM();
void saveMenuToEEPROM();
void mainDisplayFunction(unsigned int encoderValue, RenderPressMode clicked);
void altDisplayFunction(unsigned int encoderValue, RenderPressMode clicked);
void accumulateUsage();
void loadMenuChanges();
void readSensors();
void heatControl();
void acControl();
void dehumidifyControl();
void fanControl();
void resetCallback();
void configEncoderForMode();
void takeOverDisplayMain();
void takeOverDisplayMisc();
void shutDownPrevMode(bool force);
void updateBaroRiseFall();
void restartMdns();

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
void influxdbSetup();

void (*altDispRefreshFunc)(GRAPH_CNT c) = NULL;

// Main Arduino setup function
void setup()
{
    // Scope debug pin (do this first so it is available)
    pinMode(SCOPE_PIN, OUTPUT);
    SCOPE(LOW);

    // Initialize the EEPROM class to 1024 bytes of storage
    // 0x000-0x0FF  tcMenu
    // 0x100-0x3FF  local vars
    EEPROM.begin(0x400);

    // tcMenu
    setupMenu();

    // Fire up serial port
    Serial.begin(115200);

    // Splash screen
    drawSplash(5);

    // Menu timeout
    renderer.setResetIntervalTimeSeconds(5);

    // Set up the timeout callback
    renderer.setResetCallback(resetCallback);

    // Main sensor setup & initialization
    initBme280();

    // Load initial menu values
    menuMgr.load(MENU_MAGIC_KEY);
    loadMenuChanges();

    // pre-fill lastMode on startup
    lastMode = mode;

    // Read up nvm local variables
    // Should really check return code here...
    EEPROM.readBytes(EEPROM_LOCAL_VAR_ADDR, &loc, sizeof(EEPROM_LOCAL_VARS));
    EEPROM.readBytes(EEPROM_CREDENTIALS_ADDR, &creds, sizeof(EEPROM_CREDENTIALS));
    Serial.printf("EEPROM data restored\n");

    // Update boot related items in nvm. These changes will get pushed to EEPROM by
    // checkLocalVarChanges() in the main loop.
    loc.powerCycleCnt++;
    loc.bootTime = 0; // set to zero now, will update ASAP

    // Convert old EEPROM format to new format & initialize cycle counters
    // Eventually most of this can be deleted after Loren's LRTs are updated.
    if (loc.locMagic != LOC_MAGIC_KEY)
    {
        // Since this change moved where wifi and influx credentials are stored in EEPROM,
        // copy old to new location and save out.
        memcpy(creds.ssid, &loc.heatCount, 32 * 2 + 128 * 2 + 80 * 2);

        // Save old stuff to new location
        saveCredsToEEPROM();

        Serial.printf("Credentials moved to new location in EEPROM\n");

        // Set the rest of the old area to the 'erased value'. Is there a way to erase? Dunno
        memset(loc.pad0, 0xFF, 212);

        // New variables introduced with this change.
        loc.heatCount = 0;
        loc.coolCount = 0;
        loc.dhCount = 0;
        loc.locMagic = LOC_MAGIC_KEY;
    }

#if 0
    for (int i = 0; i < 256; i += 16)
    {
        char *p = (char *)&loc;
        Serial.printf("%02x: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                      i, p[i + 0], p[i + 1], p[i + 2], p[i + 3],
                      p[i + 4], p[i + 5], p[i + 6], p[i + 7],
                      p[i + 8], p[i + 9], p[i + 10], p[i + 11],
                      p[i + 12], p[i + 13], p[i + 14], p[i + 15]);
    }
    for (int i = 0; i < 512; i += 16)
    {
        char *p = (char *)&creds;
        Serial.printf("%x: %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c\n",
                      i, p[i + 0], p[i + 1], p[i + 2], p[i + 3],
                      p[i + 4], p[i + 5], p[i + 6], p[i + 7],
                      p[i + 8], p[i + 9], p[i + 10], p[i + 11],
                      p[i + 12], p[i + 13], p[i + 14], p[i + 15]);
    }
#endif

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
    pinMode(ONBOARD_LED, OUTPUT);

    // Switches
    pinMode(UP_SWITCH, INPUT_PULLUP);
    pinMode(DOWN_SWITCH, INPUT_PULLUP);
    pinMode(ENTER_SWITCH, INPUT_PULLUP);

#if PWM_TEST
    // configure LED PWM functionality
    ledcSetup(pwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(BACK_LIGHT, pwmChannel);
    ledcWrite(pwmChannel, pwmDutyCycle[PWM_MAX]); // max brightness
#else
    pinMode(BACK_LIGHT, OUTPUT);
    digitalWrite(BACK_LIGHT, TRUE);
#endif

    // Wifi
    wifiSetup();

    // Get time
    timeSetup();

    // influx
    influxdbSetup();
}

// Main Arduino control loop
void loop()
{
    uint32_t time1sec = millis() + LOOP_1_SEC;
    uint32_t time10min = 3;             // countdown
    uint32_t time24hr = T_24HOURS_IN_SEC; // countdown
    uint32_t time1min = T_1MIN_IN_SEC;
    uint32_t curTime;
    uint32_t wifiRetry = 0;
    bool wifiUp = FALSE;

    while (1)
    {
        curTime = millis();

        if (curTime >= wifiRetry)
        {
            // Poll for Wifi connection (not required to operate)
            if ((wifiUp == FALSE) && (WiFi.status() == WL_CONNECTED))
            {
                wifiUp = TRUE;
                Serial.println("wifi up -> " + WiFi.localIP().toString());
                serverSetup();

                // start/restart the mDNS server
                restartMdns();
            }

            // Check on WiFi status
            else if ((wifiUp == TRUE) && (WiFi.status() != WL_CONNECTED))
            {
                // If we get here it means we were once connected but aren't anymore...
                wifiUp = FALSE;
                WiFi.reconnect();
            }

            // The other two combinations:
            // wifiUp == FALSE && !WL_CONNECTED -- attempting connection (nothing to do here)
            // wifiUp == TRUE && WL_CONNECTED -- operational

            wifiRetry = curTime + T_100MS;
        }

        // Service tcMenu
        taskManager.runLoop();

        // Service 1 second loop timer
        if (time1sec <= curTime)
        {
            time1sec += LOOP_1_SEC;

            // flip LED
            TOGGLE_LED();

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
                    dhSeconds++;
                }
                break;

            case COOL:
                acControl();
                if (ctlState == ON)
                {
                    coolSeconds++;
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
                // Save latest data points
                cbHumd.push(HUMD_FLOAT_TO_INT(curHumd));
                cbTemp.push(TEMP_FLOAT_TO_INT(curTemp));
                cbBaro.push(BARO_FLOAT_TO_INT(curBaro));

                //                Serial.printf("t:%.2f, h:%.2f, b:%.3f\n", curTemp, curHumd, curBaro);

                // Update baro display indicator;
                // must be called after capturing a new point.
                updateBaroRiseFall();

                // call refresh function
                if (altDispRefreshFunc)
                {
                    altDispRefreshFunc(GR_0H);
                }

                time10min = T_10MIN_IN_SEC;
            }

            // Once every 24 hours, add in the control usage seconds.
            // This is delayed to minimize writes to the EEPROM.
            // Condition the save on mode != OFF.
            if (--time24hr == 0)
            {
                time24hr = T_24HOURS_IN_SEC;

                if (mode != NO_MODE)
                {
                    accumulateUsage();
                }

                // Note: the loc vars get committed to EPROM below
            }

            // Restart mDNS every minute.
            // We have to do this because of this issue:
            // https://github.com/espressif/arduino-esp32/issues/4406
            if (--time1min == 0)
            {
                time1min = T_1MIN_IN_SEC;
                if (wifiUp)
                {
                    // This is a B.S. service; very flakey. Should remove.
                    restartMdns();

                    if (influxdbUp)
                    {
                        // InFLuxDB stuff
                        // Store measured value into point
                        lrtData.clearFields();

                        // Populate data
                        lrtData.addField("humidity", curHumd);
                        lrtData.addField("pressure", curBaro);
                        lrtData.addField("temperature", curTemp);
                        lrtData.addField("state", (int32_t)ctlState);
                        lrtData.addField("mode", (int32_t)mode);

                        // Print what are we writing to influxDB
                        Serial.print("InfluxDB: ");
                        Serial.println(influxdb.pointToLineProtocol(lrtData));

                        // Write point
                        if (!influxdb.writePoint(lrtData))
                        {
                            Serial.print("InfluxDB write failed: ");
                            Serial.println(influxdb.getLastErrorMessage());
                        }
                    }
                }
            }

#if PWM_TEST
            // vary brightness up and down
            if (pwmDirUp)
            {
                pwmCount++;
                if (pwmCount > PWM_MAX)
                {
                    pwmCount = PWM_MAX - 1;
                    pwmDirUp = 0;
                }
            }
            else // pwdDirUp == 0
            {
                pwmCount--;
                if (pwmCount < 0)
                {
                    pwmCount = 1;
                    pwmDirUp = 1;
                }
            }
            ledcWrite(pwmChannel, pwmDutyCycle[pwmCount]);
#endif

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

// Save credentials to EEPROM
void saveCredsToEEPROM()
{
    EEPROM.writeBytes(EEPROM_CREDENTIALS_ADDR, &creds, sizeof(EEPROM_CREDENTIALS));
    EEPROM.commit();
    Serial.printf("Credentials saved\n");
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
    loc.heatCount += heatCount;
    loc.coolCount += coolCount;
    loc.dhCount += dhCount;

    heatSeconds = 0;
    coolSeconds = 0;
    dhSeconds = 0;
    heatCount = 0;
    coolCount = 0;
    dhCount = 0;
}

#define BARO_IDX_NOW (cbBaro.size() - 1)
#define BARO_IDX_10MIN (cbBaro.size() - 2)
#define BARO_IDX_20MIN (cbBaro.size() - 3)
#define BARO_IDX_3HR (cbBaro.size() - 19)
// This function determines rising or falling barometer so that the up or down
// arrow can be set appropriately. It also gathers and stores historical barometeric
// pressure and humidity readings for later graphing.
void updateBaroRiseFall()
{
    // Get baro int
    int32_t curBaroLatest = cbBaro[BARO_IDX_NOW];

    // See how much we have shifted in the 3 intervals
    // Note: until we actually have the proper amount of readings to compare
    // against, these will not be accurate. This shouldn't cause any problems,
    // but is something to be aware of.
    int32_t diff10min = curBaroLatest - cbBaro[BARO_IDX_10MIN];
    int32_t diff20min = curBaroLatest - cbBaro[BARO_IDX_20MIN];
    int32_t diff3hr = curBaroLatest - cbBaro[BARO_IDX_3HR];

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

    Serial.printf("curBaro: %i, b10m: %i/%hi, b20m: %i/%hi, b3hr: %i/%hi, steady<=%i, dir: %i\n",
                  curBaroLatest,
                  cbBaro[BARO_IDX_10MIN], diff10min,
                  cbBaro[BARO_IDX_20MIN], diff20min,
                  cbBaro[BARO_IDX_3HR], diff3hr,
                  baroSteady, baroDir);
}

// This function is called by the renderer every 100 mS once the display is taken over.
#define ENC_MAX 99
void mainDisplayFunction(unsigned int encoderValue, RenderPressMode clicked)
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

// This function is called by the renderer every 100 mS once the display is taken over.
void altDisplayFunction(unsigned int encoderValue, RenderPressMode clicked)
{
    static uint32_t oldEncVal = 1;

    // Only attempt graph size stuff, if displaying a graph
    if (altDispRefreshFunc)
    {
        if (encoderValue != oldEncVal)
        {
            switch (encoderValue)
            {
            case 0:
                altDispRefreshFunc(GR_6H);
                break;

            case 1:
                altDispRefreshFunc(GR_12H);
                break;

            case 2:
                altDispRefreshFunc(GR_24H);
                break;
            }

            oldEncVal = encoderValue;
        }

        graphUpdateCurVal(SN_NULL);
    }

    // Go to the menu when enter pressed
    if (clicked)
    {
        tft.setTextSize(1); // be sure to set back to default
        altDispRefreshFunc = NULL;
        oldEncVal = 1;
        renderer.giveBackDisplay();
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
    SCOPE(1);

    static bool readSensorsInit = FALSE;
    float p, t, h;

    // Create exponential filters with a weight of 10%
    static ExponentialFilter<float> tempFilter(10, 0);
    static ExponentialFilter<float> humdFilter(1, 0); // 1% -- humidity is super sensitive and fast
    static ExponentialFilter<float> baroFilter(10, 0);

    if (readSensorsInit == FALSE)
    {
        // Throw out first reading
        readBme280(&p, &t, &h);

        // Read a second time to initialize the filter
        readBme280(&p, &t, &h);
        tempFilter.SetCurrent(t);
        humdFilter.SetCurrent(h);
        baroFilter.SetCurrent(p);
        readSensorsInit = TRUE;
    }

    // Run the filter
    readBme280(&p, &t, &h);
    tempFilter.Filter(t);
    humdFilter.Filter(h);
    baroFilter.Filter(p);

    // Extract and post-process the readings
    curTemp = (tempFilter.Current() * 1.8) + 32 + tempCal;
    curHumd = humdFilter.Current() + humdCal;
    curBaro = (baroFilter.Current() / 3386.39) + baroCal;

    SCOPE(0);
}

void initBme280()
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

void readBme280(float_t *pP, float_t *pT, float_t *pH)
{
    int32_t readCnt = 3;
    int32_t reinitCnt = 3;

    // Re-init twice (see below)
    while (reinitCnt--)
    {
        // Read 3 times
        while (readCnt--)
        {
            bme.readAllSensors(pP, pT, pH);
            if ((*pP < 30000.0) || (*pP > 110000.0) ||
                (*pT < -40.0) || (*pT > 85.0) ||
                (*pH < 0.0) || (*pH > 100.0))
            {
                // Don't bother re-initing on the way out
                if (reinitCnt)
                {
                    // Re-init
                    initBme280();
                }
            }
            else
            {
                return;
            }

        } // readCnt

    } // reinitCnt

    Serial.println("Failed to read BME280!");
    return;
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
        heatCount++;
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
            coolCount++;
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

        if (compressorDelay % 30 == 0)
        {
            Serial.printf("cool: cur:%0.2f  set:%0.2f  hys(+/-):%0.2f  %s  cmpDly: %u\n",
                          curTemp, set, hysteresis, ctlState ? "ON" : "OFF", compressorDelay);
        }
    }
#endif
}

// disable this feature for now
#define CTL_DEBOUNCE_CNT 5
uint32_t ctlDebounce = CTL_DEBOUNCE_CNT;

// Add on/off logging
// Add 5-second "debounce" before transitioning on/off
void dehumidifyControl()
{
    float set = loc.dhSetPt;

    if ((ctlState == ON) && (curHumd <= (set /* - hysteresis */)))
    {
        wait = (minRunTimeDelay != 0);

        if (!wait)
        {
            if (1) //(!ctlDebounce) -- disabled until further testing
            {
                ctlState = OFF;
                DH(OFF);
                compressorDelay = COMPRESSOR_DELAY;
                ctlDebounce = CTL_DEBOUNCE_CNT;
            }
            else
            {
                ctlDebounce--;
            }
        }
    }
    else if ((ctlState == OFF) && (curHumd >= (set + hysteresis)))
    {
        wait = (compressorDelay != 0);

        if (!wait)
        {
            if (!ctlDebounce)
            {
                ctlState = ON;
                DH(ON);
                dhCount++;
                minRunTimeDelay = dhMinRunTime;
                ctlDebounce = CTL_DEBOUNCE_CNT;
            }
            else
            {
                ctlDebounce--;
            }
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

        if ((compressorDelay && (compressorDelay % 30 == 0)) || (minRunTimeDelay && (minRunTimeDelay % 30 == 0)))
        {
            Serial.printf("dh: cur:%0.2f  set:%0.2f  hys(+N/-0):%0.2f  %s  cmpDly: %u  minRT: %u\n",
                          curHumd, set, hysteresis, ctlState ? "ON" : "OFF", compressorDelay, minRunTimeDelay);
        }
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

//    Serial.printf("Exit menu: Cur Mode: %hd, setpt: %hu\n", mode, pSetPt ? *pSetPt : 777);
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
void takeOverDisplayMain()
{
    shutDownPrevMode(FALSE);
    configEncoderForMode();
    renderer.takeOverDisplay(mainDisplayFunction);
}

void takeOverDisplayMisc()
{
    // Encoder goes from 0 - 2. Encoder is set to mid-point (1)
    switches.changeEncoderPrecision(2, 1);

    renderer.takeOverDisplay(altDisplayFunction);
}

// This function is called when the menu becomes inactive.
void resetCallback()
{
    takeOverDisplayMain();
}

// The tcMenu callbacks
void CALLBACK_FUNCTION ExitCallback(int id)
{
    takeOverDisplayMain();
}

// The rest of these callbacks exist primarily to indicate that a menu
// item has been changed, so we know when to save to EEPROM.
void CALLBACK_FUNCTION ModeCallback(int id)
{
//    int32_t val = menuModeEnum.getCurrentValue();
//    Serial.printf("CB - Mode: %d\n", val);
    menuChg = TRUE;

    // Save accumulated runtime data when mode is changed to tie up loose ends.
    accumulateUsage();
}

void CALLBACK_FUNCTION FanCallback(int id)
{
//    int32_t val = menuFanEnum.getCurrentValue();
//    Serial.printf("CB - Fan: %d\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION MinRunTimeCallback(int id)
{
//    float val = menuMinRunTime.getAsFloatingPointValue();
//    Serial.printf("CB - DhMinT: %0.1f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION HumidityCalCallback(int id)
{
//    float val = menuHumidityCal.getLargeNumber()->getAsFloat();
//    Serial.printf("CB - DhCal: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION HumdHysteresisCallback(int id)
{
//    float val = menuHumdHysteresis.getLargeNumber()->getAsFloat();
//    Serial.printf("CB - DhHys: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION TempCalCallback(int id)
{
//    float val = menuTemperatureCal.getLargeNumber()->getAsFloat();
//    Serial.printf("CB - HeatCal: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION HeatingHysteresisCallback(int id)
{
//    float val = menuHeatingHysteresis.getLargeNumber()->getAsFloat();
//    Serial.printf("CB - HeatHys: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION CoolingHysteresisCallback(int id)
{
//    float val = menuCoolingHysteresis.getLargeNumber()->getAsFloat();
//    Serial.printf("CB - CoolHys: %0.2f\n", val);
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

char *formatUsageCounterWithDays(uint32_t sec, char *buf)
{
    uint32_t hours = sec / 3600;
    sec -= hours * 3600;
    uint32_t min = sec / 60;
    sec -= min * 60;

    uint32_t days = hours / 24;
    hours -= (days * 24);

    // output format: hhhh:mm:ss
    sprintf(buf, "%3u:%02u:%02u:%02u", days, hours, min, sec);

    return buf;
}

void CALLBACK_FUNCTION DisplayUsageCntrs(int id)
{
    uint16_t x;
    uint16_t y;

    accumulateUsage();
    saveLocToEEPROM();

    takeOverDisplayMisc();
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0, 2);

    char buf[32];
    ctime_r((const time_t *)&loc.lastClear, buf);
    tft.printf("Accumulated time since:\n%s\n", &buf[4]);

    tft.printf("Heat:");
    x = tft.getCursorX() + 5; // All following are based on this X
    y = tft.getCursorY();
    tft.setCursor(x, y, 2);
    tft.printf("%s", formatUsageCounter(loc.heatSeconds, buf));
    tft.setCursor(x + 80, y, 2);
    tft.printf("%u\n", loc.heatCount);

    tft.printf("A/C:");
    y = tft.getCursorY();
    tft.setCursor(x, y, 2);
    tft.printf("%s", formatUsageCounter(loc.coolSeconds, buf));
    tft.setCursor(x + 80, y, 2);
    tft.printf("%u\n", loc.coolCount);

    tft.printf("D/H:");
    y = tft.getCursorY();
    tft.setCursor(x, y, 2);
    tft.printf("%s", formatUsageCounter(loc.dhSeconds, buf));
    tft.setCursor(x + 80, y, 2);
    tft.printf("%u\n", loc.dhCount);
}

void GatherSysInfo(bool unused)
{
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 0, 2);

    uint32_t retry = 5;
    time_t now;

    while (retry-- && (now <= A_KNOWN_GOOD_TIME))
    {
        time(&now);
    }

    // Uptime
    if (now > A_KNOWN_GOOD_TIME)
    {
        char buf[32];
        uint32_t uptime = now - loc.bootTime;
        tft.printf("Uptime: %s\n", formatUsageCounterWithDays(uptime, buf));
    }
    else
    {
        tft.printf("Uptime: unknown\n");
    }

    // boot count
    tft.printf("Startup count: %u\n", loc.powerCycleCnt);

    // SSID
    tft.printf("SSID: %s\n", pSsid);

    // IP
    tft.println("IP:  " + WiFi.localIP().toString());

    // MAC
//    tft.println("MAC: " + WiFi.macAddress());

    // WiFi Strength
    if (WiFi.status() == WL_CONNECTED)
    {
        tft.println("Signal Strength: " + WiFiSignal());
    }
    else
    {
        tft.println("Signal Strength: 0%");
    }

    // FWV
    tft.printf("PCB-FW: %s-%s\n", PCB_DISP, FW_VERSION);
}

void CALLBACK_FUNCTION DisplaySysInfo(int id)
{
    //    altDispRefreshFunc = GatherSysInfo;
    takeOverDisplayMisc();
    GatherSysInfo(TRUE);
}

void CALLBACK_FUNCTION DisplayBaroGraph(int id)
{
    if (cbBaro.size())
    {
        altDispRefreshFunc = graphBaro;
        takeOverDisplayMisc();
        graphBaro(GR_12H);
    }
}

void CALLBACK_FUNCTION DisplayHmdGraph(int id)
{
    if (cbHumd.size())
    {
        altDispRefreshFunc = graphHumidity;
        takeOverDisplayMisc();
        graphHumidity(GR_12H);
    }
}

void CALLBACK_FUNCTION DisplayTempGraph(int id)
{
    if (cbTemp.size())
    {
        altDispRefreshFunc = graphTemperature;
        takeOverDisplayMisc();
        graphTemperature(GR_12H);
    }
}

void CALLBACK_FUNCTION ClearUsageCntrs(int id)
{
    altDispRefreshFunc = NULL;

    heatSeconds = 0;
    coolSeconds = 0;
    dhSeconds = 0;
    loc.heatSeconds = 0;
    loc.coolSeconds = 0;
    loc.dhSeconds = 0;

    heatCount = 0;
    coolCount = 0;
    dhCount = 0;
    loc.heatCount = 0;
    loc.coolCount = 0;
    loc.dhCount = 0;

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

void CALLBACK_FUNCTION SafeShutdown(int id)
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
//    float val = menuBaroSteadyUpLimit.getLargeNumber()->getAsFloat();
//    Serial.printf("CB - BaroSteady: %0.4f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION PressureCalCallback(int id)
{
//    float val = menuPressureCal.getLargeNumber()->getAsFloat();
//    Serial.printf("CB - PressCal: %0.2f\n", val);
    menuChg = TRUE;
}

//******************************** Display Routines *******************************************
void dispMain()
{
    const int32_t x = 47;
    const int32_t y = 42;

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
    const int32_t x = 8;
    const int32_t y = 8;

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
    const int32_t x = 100;
    const int32_t y = 8;

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
#define MODE_DISP_Y 95
void dispCoolOff()
{
    const int32_t x = MODE_DISP_X;
    const int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("OFF ", x, y, 1); // len = 4
    tft.setTextSize(2);
    tft.drawString("COOL", x, y + 13, 1);
    tft.setTextSize(1);
}

void dispCoolWait()
{
    const int32_t x = MODE_DISP_X;
    const int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("WAIT", x, y, 1); // len = 4
    tft.setTextSize(2);
    tft.drawString("COOL", x, y + 13, 1);
    tft.setTextSize(1);
}

void dispCoolOn()
{
    const int32_t x = MODE_DISP_X;
    const int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.drawString("ON  ", x, y, 1); // len = 4
    tft.setTextSize(2);
    tft.drawString("COOL", x, y + 13, 1);
    tft.setTextSize(1);
}

void dispHeatOff()
{
    const int32_t x = MODE_DISP_X;
    const int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("OFF", x, y, 1); // len = 3
    tft.setTextSize(2);
    tft.drawString("HEAT", x, y + 13, 1);
    tft.setTextSize(1);
}

void dispHeatOn()
{
    const int32_t x = MODE_DISP_X;
    const int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("ON ", x, y, 1); // len = 3
    tft.setTextSize(2);
    tft.drawString("HEAT", x, y + 13, 1);
    tft.setTextSize(1);
}

void dispDhOff()
{
    const int32_t x = MODE_DISP_X;
    const int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("OFF ", x, y, 1); // len = 4
    tft.setTextSize(2);
    tft.drawString("DHM", x, y + 13, 1);
    tft.setTextSize(1);
}

void dispDhWait()
{
    const int32_t x = MODE_DISP_X;
    const int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("WAIT", x, y, 1); // len = 4
    tft.setTextSize(2);
    tft.drawString("DHM", x, y + 13, 1);
    tft.setTextSize(1);
}

void dispDhOn()
{
    const int32_t x = MODE_DISP_X;
    const int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.drawString("ON  ", x, y, 1); // len = 4
    tft.setTextSize(2);
    tft.drawString("DHM", x, y + 13, 1);
    tft.setTextSize(1);
}

void dispModeOff()
{
    const int32_t x = MODE_DISP_X;
    const int32_t y = MODE_DISP_Y;

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("OFF", x, y + 11, 2);
}

//****************************************** Fan **********************************************
#define FAN_DISP_X 8
#define FAN_DISP_Y 57
void dispFanOn()
{
    const int32_t x = FAN_DISP_X;
    const int32_t y = FAN_DISP_Y;

    tft.setTextColor(TFT_VIOLET, TFT_BLACK);
    tft.drawString("FAN    ", x, y, 1);
    tft.drawString("ON  ", x, y + 10, 1);
}

void dispFanOff()
{
    const int32_t x = FAN_DISP_X;
    const int32_t y = FAN_DISP_Y;

    tft.setTextColor(TFT_VIOLET, TFT_BLACK);
    tft.drawString("       ", x, y, 1);
    tft.drawString("    ", x, y + 10, 1);
}

//****************************************** Baro *********************************************
void dispBaro()
{
    const int32_t digits = 2;
    const int32_t x = 85;
    const int32_t y = 108;

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

void timeSetup()
{
    // init and get the time
    configTzTime(timeZone, "pool.ntp.org", "time.nist.gov");
}

//*********************************************************************************************
void wifiSetup()
{
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);

    // not sure either of these settings work...
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);

    // The credentials to use is based upon the state of the UP and ENTER switches
    // at boot time, according to the followingn table:
    //
    //      ENTER       UP(date)        ACTION
    //      no          no              default, use credentials from EEPROM**
    //      no          yes             same as default (no new action)
    //      yes         no              use FW credentials, do NOT UPdate EEPROM
    //      yes         yes             use FW credentials, DO UPdate EEPROM
    //
    // **If no credentials in EEPROM, use FW credentials and store them to EEPROM.
    //   This should be a one time occurance only.

    // Check if anything stored in EEPROM
    if (creds.ssid[0] == 0xFF)
    {
        // EEPROM is empty, store fw credentials to EEPROM.
        // This is the one time occurance mentioned above.
        Serial.println("Copy FW WiFi creds to EEPROM - first time");

        strncpy(creds.ssid, ssid, sizeof(creds.ssid) - 1);
        strncpy(creds.password, password, sizeof(creds.password) - 1);

        // Make sure the field is null terminated so we don't have a runaway buffer.
        // This only needs to be done once as well, as long as we only copy at most
        // "sizeof(a)-1" bytes.
        creds.ssid[sizeof(creds.ssid) - 1] = 0;         // make sure null terminated
        creds.password[sizeof(creds.password) - 1] = 0; // make sure null terminated
    }

    // Now start normal checks for button presses
    if (!digitalRead(ENTER_SWITCH))
    {
        if (!digitalRead(UP_SWITCH))
        {
            Serial.println("Copy FW WiFi creds to EEPROM - UP pressed");

            // Copy to EEPROM (UPdate)
            strncpy(creds.ssid, ssid, sizeof(creds.ssid) - 1);
            strncpy(creds.password, password, sizeof(creds.password) - 1);
        }
        Serial.println("Using FW WiFi creds");
        pSsid = (char *)ssid;
        WiFi.begin(ssid, password);
    }
    else
    {
        Serial.println("Using EEPROM WiFi creds");
        pSsid = creds.ssid;
        WiFi.begin(creds.ssid, creds.password);
    }

    Serial.printf("Connecting to: %s\n", pSsid);
}

//*********************************************************************************************
void copyInfluxCredsToEEPROM()
{
    strncpy(creds.influxDbUrl, INFLUXDB_URL, sizeof(creds.influxDbUrl));
    strncpy(creds.influxDbOrg, INFLUXDB_ORG, sizeof(creds.influxDbOrg));
    strncpy(creds.influxDbBucket, INFLUXDB_BUCKET, sizeof(creds.influxDbBucket));
    strncpy(creds.influxDbToken, INFLUXDB_TOKEN, sizeof(creds.influxDbToken));

    // Make sure the field is null terminated so we don't have a runaway buffer.
    // This only needs to be done once as well, as long as we only copy at most
    // "sizeof(a)-1" bytes.
    creds.influxDbUrl[sizeof(creds.influxDbUrl) - 1] = 0;
    creds.influxDbOrg[sizeof(creds.influxDbOrg) - 1] = 0;
    creds.influxDbBucket[sizeof(creds.influxDbBucket) - 1] = 0;
    creds.influxDbToken[sizeof(creds.influxDbToken) - 1] = 0;

    // Save it out
    saveCredsToEEPROM();
}

void influxdbSetup()
{
    bool copy = 0;

    // The influxdb credentials to use is based upon the state of the UP and ENTER switches
    // at boot time, according to the followingn table:
    //
    //      ENTER       UP(date)        ACTION
    //      no          no              default, use credentials from EEPROM**
    //      no          yes             same as default (no new action)
    //      yes         no              use FW credentials, do NOT UPdate EEPROM
    //      yes         yes             use FW credentials, DO UPdate EEPROM
    //
    // **If no credentials in EEPROM, use FW credentials and store them to EEPROM.
    //   This should be a one time occurance only.

    // Check if anything stored in EEPROM
    if (*(uint32_t *)(&creds.influxDbToken) == UINT32_ERASED_VALUE)
    {
        Serial.println("Copy FW influx creds to EEPROM - first time");
        copyInfluxCredsToEEPROM();
        copy = 1;
    }
    else
    {
        // Now start normal checks for button presses
        if (!digitalRead(ENTER_SWITCH))
        {
            if (!digitalRead(UP_SWITCH))
            {
                Serial.println("Copy FW influx creds to EEPROM - UP pressed");

                // Copy to EEPROM (UPdate)
                copyInfluxCredsToEEPROM();
                copy = 1;
            }
        }
    }

    if (copy)
    {
        Serial.println("Using FW influx creds");
    }
    else
    {
        Serial.println("Using EEPROM influx creds");
    }

    // Serial.println(creds.influxDbUrl);

    // Setup connection
    if (strcmp(INFLUXDB_URL, "URL") != 0)
    {
        influxdb.setConnectionParams(creds.influxDbUrl, creds.influxDbOrg, creds.influxDbBucket,
                                     creds.influxDbToken, InfluxDbCloud2CACert);
        influxdbUp = TRUE;
    }
    else
    {
        Serial.println("No influx creds available");
    }
}

// start mDNS server
void restartMdns()
{
    // Remove previous instance
    MDNS.end();

    // Start
    if (!MDNS.begin("LRT-basement"))
    {
        Serial.println("Error starting mDNS");
    }
    else
    {
        // Add service to MDNS-SD
        MDNS.addService("http", "tcp", 80);
    }
}
