// Copyright © 2022 Randy Rysavy <randy.rysavy@gmail.com>
// This work is free. You can redistribute it and/or modify it under the
// terms of the Do What The Fuck You Want To Public License, Version 2,
// as published by Sam Hocevar. See http://www.wtfpl.net/ for more details.

#include <CircularBuffer.h>

// Logical defines
#define FALSE 0
#define TRUE 1
#define OFF FALSE
#define ON TRUE

extern float curTemp; // BME280
extern float curHumd; // BME280
extern float curBaro; // BME280

// The following variables are loaded from the menu
// START of tcMenu loaded variables
extern float tempCal;    // calibration factor
extern float humdCal;    // calibration factor
extern float baroCal;    // calibration factor
extern float hysteresis; //  (+/-) hysteresis/2 is centered around the set point.

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
} EEPROM_LOCAL_VARS;
extern EEPROM_LOCAL_VARS loc; // local working variables

// Wifi and web server stuff
extern void serverSetup();

// Other function prototypes
void graphBaro(boolean drawGrid);
void graphHumidity(boolean drawGrid);
void graphTemperature(boolean drawGrid);

#define HIST_CNT (12 * 6) // every 10 min, 12 hrs total
//#define HIST_CNT (24 * 6) // every 10 min, 24 hrs total
extern CircularBuffer<uint16_t, HIST_CNT> cbBaro;
extern CircularBuffer<int16_t, HIST_CNT> cbHumd;
extern CircularBuffer<int16_t, HIST_CNT> cbTemp;
