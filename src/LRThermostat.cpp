#include <Arduino.h>
#include "LRThermostat_menu.h"
#include <Adafruit_BME280.h>
#include <EEPROM.h>
//#include <WiFi.h>

#define MENU_MAGIC_KEY 0xB00B
#define EEPROM_LOCAL_VAR_ADDR 0x100 // Leave the lower half for menu storage
#define INACTIVITY_TIMEOUT 10000    // 10000 mS = 10 sec
#define LOOP_1_SEC 1000             // 1000 mS = 1 sec

// Dehumidifier minimum run time once switched on, to avoid short cycling.
// TODO: add to menu
#define DH_MIN_RUN_TIME (30 * 60 * 1000) // min * 60 sec/min * 1000 mS/s

#define FAN_RELAY 16  // GPIO
#define HEAT_RELAY 17 // GPIO

#define FALSE 0
#define TRUE 1
#define OFF 0
#define ON 1
#define FAN(a) digitalWrite(FAN_RELAY, ((a) ? (OFF) : (ON)))
#define HEAT(a) digitalWrite(HEAT_RELAY, ((a) ? (OFF) : (ON)))

// Globals
Adafruit_BME280 bme;

float curTemp = 0; // BME280
float curHumd = 0; // BME280
float curBaro = 0; // BME280

float tempCal = 0.0; // calibration factor
float humdCal = 0.0; // calibration factor
float baroCal = 0.0; // calibration factor

// Note: tcMenu does not provide an enums like this. Be sure to update these
// if/when you change the mode variable in tcMenu. Order must match!
typedef enum
{
    NO_MODE,
    HEAT,
    COOL,
    DEHUMIDIFY
} MODE;
MODE mode;

typedef enum
{
    FAN_ON,
    FAN_AUTO
} FAN;
FAN fan;

bool ctlState = OFF;
bool menuChg = FALSE;
uint32_t lclVarChgTime = 0;
uint32_t dhMinRunTime = 0;
uint32_t dhRunUntil = 0;

float hysteresis;       //  +/- hysteresis/2 is centered around the set point.

typedef struct
{
    int16_t heatSetPt; // heat  0-100 is range for all setPts
    int16_t acSetPt;   // A/C
    int16_t dhSetPt;   // dehumidifier
    int16_t pad0;      // pad out to 4 bytes
} EEPROM_LOCAL_VARS;

EEPROM_LOCAL_VARS loc;            // local working variables
EEPROM_LOCAL_VARS chgdVars = {0}; // Changes to be committed

// Function declarations
void checkLocalVarChanges();
void myDisplayFunction(unsigned int encoderValue, RenderPressMode clicked);
void loadMenuChanges();
void readSensors();
void heatControl();
void acControl();
void dehumidifyControl();
void myResetCallback();

// display func declarations
void mainTemp();
void tempSmall();
void tempSetPt();
void coolOn();
void coolOff();
void heatOn();
void heatOff();
void dhOn();
void dhOff();
void modeOff();
void fanOn();
void fanOff();
void humiditySmall();
void humidityBig();
void humiditySetPt();

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

    // Should really check return code here...
    EEPROM.readBytes(EEPROM_LOCAL_VAR_ADDR, &loc, sizeof(EEPROM_LOCAL_VARS));
    Serial.printf("Menu+local data restored, 0x%04X\n", (uint32_t)EEPROM.readUShort(0x0));

    // Setup the rest of the hardware
    pinMode(FAN_RELAY, OUTPUT);
    FAN(OFF);
    pinMode(HEAT_RELAY, OUTPUT);
    HEAT(OFF);
}

// Main Arduino control loop
void loop()
{
    uint32_t time1sec = 0;
    uint32_t curTime;

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
                break;

            case DEHUMIDIFY:
                dehumidifyControl();
                break;

            case COOL:
                acControl();
                break;

            case NO_MODE:
            default:
                // make sure control relays are off. Fan on is OK.
                HEAT(OFF);
            }

            //            // debug
            //            Serial.printf("%u\n", lclVarChgTime);

            // Check for local var changes that need EEPROM update
            checkLocalVarChanges();
        }

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

//
// This function is called by the renderer every 100 mS once the display is taken over.
//
void myDisplayFunction(unsigned int encoderValue, RenderPressMode clicked)
{
    static bool myDispInit = FALSE;
    //    char s[20];

    // No need to call this stuff more than once or it causes screen flicker
    if (myDispInit == FALSE)
    {
        // The encoderValue seems to change opposite of what I expect, so we
        // need to convert to the reverse.
        switches.changeEncoderPrecision(999, 100 - loc.heatSetPt);
        encoderValue = 100 - loc.heatSetPt;

        // Display init
        tft.fillScreen(TFT_BLACK);
        //        tft.drawRect(0, 0, 160, 128, TFT_RED);
        //        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        //        tft.drawString("O", 110, 35, 2);

        myDispInit = TRUE;
    }

    // Reload menu values
    if (menuChg)
    {
        loadMenuChanges();

        // save menu to EEPROM
        menuMgr.save(MENU_MAGIC_KEY);
        EEPROM.commit();

        Serial.printf("Menu data saved, 0x%04X\n", (uint32_t)EEPROM.readUShort(0x0));
    }

    // Update setpoint if different; same encoder conversion here
    loc.heatSetPt = 100 - encoderValue;

    // Items that get updated
    //    tft.drawNumber(round(curTemp), 45, 40, 7);
    //    sprintf(s, "Setting: %d", loc.heatSetPt);
    //    tft.drawCentreString(s, 80, 105, 2);
    mainTemp();
    humiditySmall();

    switch (mode)
    {
    case HEAT:
        tempSetPt();
        if (ctlState == ON)
        {
            heatOn();
        }
        else
        {
            heatOff();
        }
        break;

    case COOL:
        tempSetPt();
        if (ctlState == ON)
        {
            coolOn();
        }
        else
        {
            coolOff();
        }
        break;

    case DEHUMIDIFY:
        if (ctlState == ON)
        {
            dhOn();
        }
        else
        {
            dhOff();
        }
        break;

    default:
    case NO_MODE:
        modeOff();
        break;
    }

    if (fan == FAN_ON)
    {
        fanOn();
    }
    else
    {
        fanOff();
    }

    // Go to the menu when enter pressed
    if (clicked)
    {
        tft.setTextSize(1); // be sure to set back to default
        renderer.giveBackDisplay();
        myDispInit = FALSE;
    }
}

void loadMenuChanges()
{
    tempCal = menuTempCal.getLargeNumber()->getAsFloat();
    humdCal = menuHumidityCal.getLargeNumber()->getAsFloat();
    //    baroCal = menuPressureCal.getLargeNumber()->getAsFloat();
    mode = (MODE)menuModeEnum.getCurrentValue();
    fan = (FAN)menuFanEnum.getCurrentValue();
    dhMinRunTime = menuMinRunTime.getAsFloatingPointValue();

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

    menuChg = FALSE;
}

void readSensors()
{
    curTemp = bme.readTemperature() * 1.8 + 32 + tempCal;
    curHumd = bme.readHumidity() + humdCal;
    curBaro = bme.readPressure() / 3386.39 + baroCal;
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

#if 1
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

// TODO: Need compressor protection (5 min between on -> off -> on transistion)
void acControl()
{
    float set = loc.acSetPt; 

    if ((ctlState == ON) && (curTemp < (set - hysteresis)))
    {
        ctlState = OFF;
        HEAT(OFF);
    }
    else if ((ctlState == OFF) && (curTemp > (set + hysteresis)))
    {
        ctlState = ON;
        HEAT(ON);
    }
    else
    {
        // You're in-between, do nothing
    }

#if 1
    static bool lastSt = OFF;
    static float lastSet = -1;

    if ((lastSt != ctlState) || (lastSet != set))
    {
        lastSt = ctlState;
        lastSet = set;

        Serial.printf("cool: cur:%0.2f  set:%0.2f  hys(+/-):%0.2f  %s\n",
                      curTemp, set, hysteresis, ctlState ? "ON" : "OFF");
    }
#endif
}

void dehumidifyControl()
{
#if 1
    float set = loc.dhSetPt;

    if ((ctlState == ON) && (curHumd > (set + hysteresis)))
    {
        ctlState = OFF;
        HEAT( OFF );  // DH uses the heat relay
    }
    else if ((ctlState == OFF) && (curHumd < (set - hysteresis)))
    {
        ctlState = ON;
        HEAT( ON );

        if (!dhRunUntil)
        {
            dhRunUntil = millis() + DH_MIN_RUN_TIME;
        }
    }
    else
    {
        // You're in-between, do nothing
    }

#if 1
    static bool lastSt = OFF;
    static float lastSet = -1;

    if ((lastSt != ctlState) || (lastSet != set))
    {
        lastSt = ctlState;
        lastSet = set;

        Serial.printf("dh: cur:%0.2f  set:%0.2f  hys(+/-):%0.2f  %s\n", 
                curHumd, set, hysteresis, ctlState ? "ON" : "OFF");
    }
#endif
#endif
}

// This function is called when the menu becomes inactive.
void myResetCallback()
{
    renderer.takeOverDisplay(myDisplayFunction);
}

void CALLBACK_FUNCTION ExitMenuCallback(int id)
{
    renderer.takeOverDisplay(myDisplayFunction);
}

void CALLBACK_FUNCTION ModeCallback(int id)
{
    int32_t val = menuModeEnum.getCurrentValue();
    Serial.printf("Mode: %d\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION FanCallback(int id)
{
    int32_t val = menuFanEnum.getCurrentValue();
    Serial.printf("Fan: %d\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION HomePresetCallback(int id)
{
    float val = menuHomePreset.getAsFloatingPointValue();
    Serial.printf("HomePre: %0.1f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION AwayPresetCallback(int id)
{
    float val = menuAwayPreset.getAsFloatingPointValue();
    Serial.printf("AwayPre: %0.1f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION MinRunTimeCallback(int id)
{
    float val = menuMinRunTime.getAsFloatingPointValue();
    Serial.printf("DhMinT: %0.1f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION HumidityCalCallback(int id)
{
    float val = menuHumidityCal.getLargeNumber()->getAsFloat();
    Serial.printf("DhCal: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION HumdHysteresisCallback(int id)
{
    float val = menuHumdHysteresis.getLargeNumber()->getAsFloat();
    Serial.printf("DhHys: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION TempCalCallback(int id)
{
    float val = menuTempCal.getLargeNumber()->getAsFloat();
    Serial.printf("HeatCal: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION TempHysteresisCallback(int id)
{
    float val = menuTempHysteresis.getLargeNumber()->getAsFloat();
    Serial.printf("HeatHys: %0.2f\n", val);

#if 0
    char *pN = (char *)menuTempHysteresis.getLargeNumber();
 
    int i;
    for (i=0; i<32; i++)
    {
        Serial.printf("%02X ", pN[i]);
    }
    Serial.println();    
#endif

    menuChg = TRUE;
}

void CALLBACK_FUNCTION CoolingCalCallback(int id)
{
    float val = menuCoolingCal.getLargeNumber()->getAsFloat();
    Serial.printf("CoolCal: %0.2f\n", val);
    menuChg = TRUE;
}

void CALLBACK_FUNCTION CoolingHysteresisCallback(int id)
{
    float val = menuCoolingHysteresis.getLargeNumber()->getAsFloat();
    Serial.printf("CoolHys: %0.2f\n", val);
    menuChg = TRUE;
}

void mainTemp()
{
    tft.setTextColor(TFT_CYAN, TFT_BLACK);     // Note: the new fonts do not draw the background colour
    tft.drawNumber(round(curTemp), 43, 45, 7); // Temperature using font 7
    tft.drawString("O", 110, 35, 2);
}

//****************************************** Temperature Set Point ****************************
void tempSetPt()
{
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("SET", 100, 8, 1);
    tft.setTextSize(2);
    tft.drawNumber((uint32_t)loc.heatSetPt, 123, 8, 1); // font 1
    tft.setTextSize(1);
    tft.drawString(" o", 145, 6, 1);
}

// ****************************** Humidity Reading for when temperature is main screen ****************
void humiditySmall()
{
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawNumber(round(curHumd), 8, 8, 1); // Humidity at font 1
    tft.setTextSize(1);
    tft.drawString(" %", 30, 6, 1);
}

//****************************************** MODES *****************************************************
void coolOff()
{
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("COOL    ", 8, 115, 1);
    tft.drawString("OFF", 8, 105, 1);
}

void coolOn()
{
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.drawString("COOL    ", 8, 115, 1);
    tft.drawString("ON ", 8, 105, 1);
}

void heatOff()
{
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("HEAT    ", 8, 115, 1);
    tft.drawString("OFF", 8, 105, 1);
}

void heatOn()
{
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("HEAT    ", 8, 115, 1);
    tft.drawString("ON ", 8, 105, 1);
}

void dhOff()
{
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("DEHUMIDIFY", 8, 115, 1);
    tft.drawString("OFF", 8, 105, 1);
}

void dhOn()
{
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.drawString("DEHUMIDIFY", 8, 115, 1);
    tft.drawString("ON ", 8, 105, 1);
}

void modeOff()
{
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("          ", 8, 105, 1);
    tft.drawString("OFF       ", 8, 115, 1);
}

//****************************************** Fan ******************************************************
void fanOn()
{
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("FAN    ", 135, 115, 1);
    tft.drawString("ON  ", 135, 105, 1);
}

void fanOff()
{
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("       ", 135, 115, 1);
    tft.drawString("    ", 135, 105, 1);
}

// ****************************** Humidity Reading for when Humidity is main screen ****************
void humidityBig()
{
    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawNumber(round(curHumd), 43, 45, 7);
    tft.setTextSize(2);
    tft.drawString(" %", 103, 40, 1);
    tft.setTextSize(1);
}

// ****************************** Humidity set point ***********************************************
void humiditySetPt()
{
    tft.setTextColor(TFT_PINK, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawNumber((uint32_t)loc.dhSetPt, 8, 8, 1); // Humidity at font 1
    tft.setTextSize(1);
    tft.drawString(" %", 30, 6, 1);
    tft.drawString("SET", 48, 8, 1);
}

//******************************** Temperature in Dehumidity Mode ***********************************
void tempSmall()
{
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawNumber(round(curTemp), 123, 8, 1); // font 1
    tft.setTextSize(1);
    tft.drawString(" o", 145, 6, 1);
}

