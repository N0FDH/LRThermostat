// Copyright © 2022 Randy Rysavy <randy.rysavy@gmail.com>
// This work is free. You can redistribute it and/or modify it under the
// terms of the Do What The Fuck You Want To Public License, Version 2,
// as published by Sam Hocevar. See http://www.wtfpl.net/ for more details.

#ifndef LRTVERSION_H
#define LRTVERSION_H

// Uncomment only ONE of the following.
// This selects what type of hardware you are targeting.
//============================
#define PROTOTYPE // All prototypes
// #define PCB_V1_GREENTAB2 // Loren's basement
// #define PCB_V1_GREENTAB3 // Loren's garage
// #define PCB_V1_REDTAB // Randy's basement & garage
//============================

// Special reconfiguration for Randy's cabin (garage) heat and dehumidifier controller
#define DHM_USES_AC_RELAY

// FW version
#define FW_VERSION "1.61"

//==============================================================================
// *** DON'T CHANGE ANYTHING BELOW THIS LINE ***
//==============================================================================

#ifdef PROTOTYPE
#define PCB_VERSION 0 // 0 = prototype, 1 = 1st main release, etc
#define ST7735_GREENTAB2
#define PCB_DISP "0G"
#endif

#ifdef PCB_V1_GREENTAB2
#define PCB_VERSION 1 // 0 = prototype, 1 = 1st main release, etc
#define ST7735_GREENTAB2
#define PCB_DISP "1G"
#endif

#ifdef PCB_V1_GREENTAB3
#define PCB_VERSION 1 // 0 = prototype, 1 = 1st main release, etc
#define ST7735_GREENTAB3
#define PCB_DISP "1H"
#endif

#ifdef PCB_V1_REDTAB
#define PCB_VERSION 1 // 0 = prototype, 1 = 1st main release, etc
#define ST7735_REDTAB
#define PCB_DISP "1R"
#endif

#endif // LRTVERSION_H
