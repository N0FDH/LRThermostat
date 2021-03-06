// Copyright © 2022 Randy Rysavy <randy.rysavy@gmail.com>
// This work is free. You can redistribute it and/or modify it under the
// terms of the Do What The Fuck You Want To Public License, Version 2,
// as published by Sam Hocevar. See http://www.wtfpl.net/ for more details.

#ifndef LRTHERMOSTAT_H
#define LRTHERMOSTAT_H

#include <CircularBuffer.h>
#include "LRTVersion.h"

// Hardware definitions
#define SCOPE_PIN 27        // GPIO
#define FAN_RELAY 16        // GPIO
#define HEAT_RELAY 17       // GPIO
#define AC_RELAY 18         // GPIO
#define BACK_LIGHT 25       // GPIO
#define DH_RELAY HEAT_RELAY // DH always uses HEAT relay
#define ENTER_SWITCH 4      // GPIO

#if (PCB_VERSION == 0)
#define UP_SWITCH 13   // GPIO
#define DOWN_SWITCH 15 // GPIO
#else
#define UP_SWITCH 35   // GPIO
#define DOWN_SWITCH 34 // GPIO
#endif

// Logical defines
#define FALSE 0
#define TRUE 1
#define OFF FALSE
#define ON TRUE

// Scope debug pin
#define SCOPE(a) digitalWrite(SCOPE_PIN, ((a) ? (ON) : (OFF)))

extern float curTemp; // BME280
extern float curHumd; // BME280
extern float curBaro; // BME280

// The following variables are loaded from the menu
// START of tcMenu loaded variables
extern float tempCal;    // calibration factor
extern float humdCal;    // calibration factor
extern float baroCal;    // calibration factor
extern float hysteresis; //  (+/-) hysteresis/2 is centered around the set point.

extern bool ctlState; // heat/cool/dh state
extern bool fanState; // fan state

// Note: tcMenu does not provide enums like this. Be sure to update these
// if/when you change the 'mode' or 'fan' variable in tcMenu. Order must match!
typedef enum
{
    NO_MODE,   // 0
    HEAT,      // 1
    COOL,      // 2
    DEHUMIDIFY // 3
} MODE;

typedef enum
{
    FAN_ON,
    FAN_AUTO
} FAN;

extern MODE mode, lastMode;
extern FAN fan;
// END of tcMenu loaded variables

// The local variables that are backed up in EEPROM
typedef struct
{
    int16_t heatSetPt;      // heat  0-100 is range for all setPts
    int16_t acSetPt;        // A/C
    int16_t dhSetPt;        // dehumidifier
    uint16_t powerCycleCnt; // put something in an otherwise empty pad :)
    uint32_t bootTime;      // epoch time() of boot
    uint32_t lastClear;     // epoch time() of last clear of "on time" counters

    uint32_t heatSeconds;   // total heat "on" time
    uint32_t coolSeconds;   // total a/c "on" time
    uint32_t dhSeconds;     // total dh "on" time
    uint32_t pad1;          // pad to 32-byte alignment

    char ssid[32];     // WiFi
    char password[32]; // WiFi
} EEPROM_LOCAL_VARS;
extern EEPROM_LOCAL_VARS loc; // local working variables

// Wifi and web server stuff
extern void serverSetup();
extern String WiFiSignal();

// Other function prototypes
typedef enum
{
    SN_NULL,
    SN_BARO,
    SN_TEMP,
    SN_HUMD
} SENSOR_TYPE;
typedef enum
{
    GR_0H = 0,
    GR_6H = 36, 
    GR_12H = 72,
    GR_24H = 144
} GRAPH_CNT;

// Call with drawGrid == true. The coordinate system will only be drawn once.
void graphUpdateCurVal(SENSOR_TYPE type);
void drawGraph(SENSOR_TYPE type, GRAPH_CNT count);
inline void graphBaro(GRAPH_CNT count)
{
    drawGraph(SN_BARO, count);
}

inline void graphTemperature(GRAPH_CNT count)
{
    drawGraph(SN_TEMP, count);
}

inline void graphHumidity(GRAPH_CNT count)
{
    drawGraph(SN_HUMD, count);
}

#define HIST_CNT (24 * 6) // every 10 min, 24 hrs total
extern CircularBuffer<int16_t, HIST_CNT> cbBaro;
extern CircularBuffer<int16_t, HIST_CNT> cbHumd;
extern CircularBuffer<int16_t, HIST_CNT> cbTemp;

#endif // LRTHERMOSTAT_H
