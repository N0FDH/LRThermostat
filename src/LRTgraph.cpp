// Copyright © 2022 Randy Rysavy <randy.rysavy@gmail.com>
// This work is free. You can redistribute it and/or modify it under the
// terms of the Do What The Fuck You Want To Public License, Version 2,
// as published by Sam Hocevar. See http://www.wtfpl.net/ for more details.
//
// This function provides a Cartesian graph implementation.
// The original "graph()" function was written by Kris Kasprzak:
// https://github.com/KrisKasprzak/Graphing

#include <Arduino.h>
#include "LRThermostat.h"
#include "tcMenu/LRThermostat_menu.h"

void graph(TFT_eSPI &d,
           float_t x, float_t y,
           int32_t gx, int32_t gy, int32_t w, int32_t h,
           float_t xlo, float_t xhi, float_t xinc,
           float_t ylo, float_t yhi, float_t yinc,
           String title, String xlabel, String ylabel,
           uint32_t gcolor, uint32_t acolor, uint32_t pcolor,
           uint32_t tcolor, uint32_t bcolor,
           boolean &redraw, uint32_t digits);

void drawSetpointLine(TFT_eSPI &d,
                      float_t y,
                      int32_t gx, int32_t gy,
                      int32_t w, int32_t h,
                      float_t xlo, float_t xhi,
                      float_t ylo, float_t yhi,
                      uint32_t tcolor);

// Not used, but this does work. remember to negate 'h' if you are using it. I don't
// think it buys anything by using it. The big, clunky floats gotta be loaded and "mathed"
// no matter how its done.
// float_t transform(float_t *p1, float_t *p2, int32_t *p3, float_t *p4, int32_t *p5)
// {
//     return (*p1 - *p2) * ((float_t)(*p3)) / (*p4 - *p2) + ((float_t)(*p5));
// }

#define LTBLUE 0xB6DF
#define LTTEAL 0xBF5F
#define LTGREEN 0xBFF7
#define LTCYAN 0xC7FF
#define LTRED 0xFD34
#define LTMAGENTA 0xFD5F
#define LTYELLOW 0xFFF8
#define LTORANGE 0xFE73
#define LTPINK 0xFDDF
#define LTPURPLE 0xCCFF
#define LTGREY 0xE71C

#define BLUE 0x001F
#define TEAL 0x0438
#define GREEN 0x07E0
#define CYAN 0x07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define ORANGE 0xFC00
#define PINK 0xF81F
#define PURPLE 0x8010
#define GREY 0xC618
#define WHITE 0xFFFF
#define BLACK 0x0000

#define DKBLUE 0x000D
#define DKTEAL 0x020C
#define DKGREEN 0x03E0
#define DKCYAN 0x03EF
#define DKRED 0x6000
#define DKMAGENTA 0x8008
#define DKYELLOW 0x8400
#define DKORANGE 0x8200
#define DKPINK 0x9009
#define DKPURPLE 0x4010
#define DKGREY 0x4A49

//*****************************************************************************
// Note: X increases left to right
//       Y increases top to bottom
#define WIDTH_X 110
#define HEIGHT_Y 95

#define GRAPH_UL_X 40
#define GRAPH_UL_Y 17

#define GRAPH_LL_X GRAPH_UL_X
#define GRAPH_LL_Y (GRAPH_UL_Y + HEIGHT_Y)

#define TITLE_UL_X GRAPH_UL_X
#define TITLE_UL_Y 4

#define XNUMS_UL_X GRAPH_UL_X
#define XNUMS_UL_Y 117

#define GRID_COLOR DKBLUE
#define AXIS_COLOR DKBLUE
#define DATA_COLOR YELLOW
#define SETPT_COLOR RED
#define TEXT_COLOR WHITE
#define BG_COLOR BLACK

#define XLOW 0
#define XHI HIST_CNT
#define XINC (XHI / 6)

// Current val after title
#define CUR_VAL_X 115
#define CUR_VAL_Y TITLE_UL_Y

//*****************************************************************************

void graphUpdateCurVal(SENSOR_TYPE type)
{
    static SENSOR_TYPE lastType = SN_NULL;

    if (type == SN_NULL)
    {
        type = lastType;
    }
    else
    {
        lastType = type;
    }

    switch (type)
    {
    case SN_BARO:

        tft.setTextColor(DATA_COLOR, BG_COLOR);
        tft.drawFloat(curBaro, 2, CUR_VAL_X, CUR_VAL_Y, 1);
        break;

    case SN_TEMP:

        tft.setTextColor(DATA_COLOR, BG_COLOR);
        tft.drawNumber(round(curTemp), (CUR_VAL_X + 10), CUR_VAL_Y, 1);
        tft.drawString("o", (CUR_VAL_X + 10) + 12, CUR_VAL_Y - 3, 1);
        break;

    case SN_HUMD:

        tft.setTextColor(DATA_COLOR, BG_COLOR);
        tft.drawNumber(round(curHumd), CUR_VAL_X, CUR_VAL_Y, 1);
        tft.drawString("%", CUR_VAL_X + 12, CUR_VAL_Y, 1);
        break;

    default:
        break;
    }
}

void drawGraph(SENSOR_TYPE type, GRAPH_CNT count)
{
    static GRAPH_CNT defCount = GR_12H;

    CircularBuffer<int16_t, HIST_CNT> *pData = NULL;
    const char *pTitle = NULL;
    uint32_t digits = 0;
    uint32_t yDiv = 100;
    uint32_t low = 0;
    uint32_t high = 0;
    float dhHighLimit = ((float_t)loc.dhSetPt + menuHumdHysteresis.getLargeNumber()->getAsFloat());

    // Save last count so we know what to draw if we get a '0' as count
    if (count == GR_0H)
    {
        count = defCount;
    }
    else
    {
        defCount = count;
    }

    // Set up pointer to propper data set
    switch (type)
    {
    case SN_BARO:
        pData = &cbBaro;
        break;

    case SN_TEMP:
        pData = &cbTemp;
        break;

    case SN_HUMD:
        pData = &cbHumd;
        break;

    default:
        break;
    }

    // Determine start point in data
    int32_t start = pData->size() >= count ? pData->size() - count : 0;

    // Set up other mode specific items
    switch (type)
    {
    case SN_BARO:
        pData = &cbBaro;
        pTitle = "  Baro Pres";
        digits = 2;
        yDiv = 1000;
        low = cbBaro[start];
        high = low;
        break;

    case SN_TEMP:
        pData = &cbTemp;
        pTitle = " Temperature";
        low = loc.heatSetPt * 100; // start with setpoint to be sure it is included
        high = low;
        break;

    case SN_HUMD:
        pData = &cbHumd;
        pTitle = "  Humidity";
        low = loc.dhSetPt * 100; // start with setpoint to be sure it is included
        high = (uint32_t)(dhHighLimit * 100); // DH high limit has to be included too
        break;

    default:
        break;
    }

    // Scan data to get its range
    for (int32_t n = start; n < pData->size(); n++)
    {
        uint32_t tmp = (*pData)[n];

        if (tmp < low)
        {
            low = tmp;
        }
        else if (tmp > high)
        {
            high = tmp;
        }
    }

    // Calculate data range for graph
    float_t ylo = 0;
    float_t yhi = 0;

    switch (type)
    {
    case SN_BARO:
        // Note: these round up/downs are using integer math then floating for final division
        ylo = ((float_t)(low / 100)) / 10;         // round down to nearest 0.100
        yhi = ((float_t)((high + 99) / 100)) / 10; // round up to nearest 0.100
        break;

    case SN_TEMP:
        // Note: these round up/downs are using integer math exclusively
        ylo = low / 500 * 5;          // round down to nearest 5
        yhi = (high + 499) / 500 * 5; // round up to nearest 5
        break;

    case SN_HUMD:
        // Note: these round up/downs are using integer math exclusively
        ylo = low / 1000 * 10;          // round down to nearest 10
        yhi = (high + 999) / 1000 * 10; // round up to nearest 10
        break;

    default:
        break;
    }

    tft.fillScreen(BLACK);

    boolean drawGrid = TRUE;
    uint32_t x0 = 0;
    for (int32_t x = start; x < pData->size(); x++, x0++)
    {
        float_t y = ((float_t)((*pData)[x])) / yDiv;

        graph(tft,
              (float_t)x0, y,            // data point
              GRAPH_LL_X, GRAPH_LL_Y,    // lower left corner of graph
              WIDTH_X, HEIGHT_Y,         // width, height
              XLOW, count, count / 6,    // xlow, xhi, xinc
              ylo, yhi, (yhi - ylo) / 5, // ylow, yhi, yinc
              pTitle, "", "",            // title, x-label, y-label
              GRID_COLOR,                // grid line color
              AXIS_COLOR,                // axis lines color
              DATA_COLOR,                // plotted data color
              TEXT_COLOR,                // text color
              BG_COLOR,                  // background color
              drawGrid,                  // redraw flag
              digits);                   // digits
    }

    if (type != SN_BARO)
    {
        // Draw the setpoint
        drawSetpointLine(tft,
                         type == SN_TEMP ? (float_t)loc.heatSetPt : (float_t)loc.dhSetPt, // Set point
                         GRAPH_LL_X, GRAPH_LL_Y,                                          // lower left corner of graph
                         WIDTH_X, HEIGHT_Y,                                               // width, height
                         XLOW, XHI,                                                       // xlow, xhi
                         ylo, yhi,                                                        // ylow, yhi
                         SETPT_COLOR);                                                    // setpoint line color
    }
    if (type == SN_HUMD)
    {
        // Draw the setpoint + hysteresis
        drawSetpointLine(tft,
                         dhHighLimit,            // Set point
                         GRAPH_LL_X, GRAPH_LL_Y, // lower left corner of graph
                         WIDTH_X, HEIGHT_Y,      // width, height
                         XLOW, XHI,              // xlow, xhi
                         ylo, yhi,               // ylow, yhi
                         SETPT_COLOR);           // setpoint line color
    }

    graphUpdateCurVal(type);
}

//   Function to draw a cartesian coordinate system and plot whatever data you want
//   just pass x and y and the graph will be drawn
//   huge arguement list
//   &d name of your display object
//   x = x data point
//   y = y datapont
//   gx = x graph location (lower left)
//   gy = y graph location (lower left)
//   w = width of graph
//   h = height of graph
//   xlo = lower bound of x axis
//   xhi = upper bound of x asis
//   xinc = division of x axis (distance not count)
//   ylo = lower bound of y axis
//   yhi = upper bound of y asis
//   yinc = division of y axis (distance not count)
//   title = title of graph
//   xlabel = x axis label
//   ylabel = y axis label
//   gcolor = graph line colors
//   acolor = axis line colors
//   pcolor = color of your plotted data
//   tcolor = text color
//   bcolor = background color
//   &redraw = flag to redraw graph on first call only
//   digits = number of significant digits after decimal point on y-axis labels
void graph(TFT_eSPI &d,
           float_t x, float_t y,
           int32_t gx, int32_t gy, int32_t w, int32_t h,
           float_t xlo, float_t xhi, float_t xinc,
           float_t ylo, float_t yhi, float_t yinc,
           String title, String xlabel, String ylabel,
           uint32_t gcolor, uint32_t acolor, uint32_t pcolor,
           uint32_t tcolor, uint32_t bcolor,
           boolean &redraw, uint32_t digits)
{
    static float_t ox, oy;

    float_t i;
    float_t temp;

    if (redraw == true)
    {
        redraw = false;
        ox = (x - xlo) * ((float_t)w) / (xhi - xlo) + (float_t)gx;
        oy = (y - ylo) * ((float_t)-h) / (yhi - ylo) + (float_t)gy;

        // Draw y scale
        //*****************************************************************************
        // add a tiny amount to yhi to be sure to get last line
        for (i = ylo; i <= (yhi + .0001); i += yinc)
        {
            // compute the transform
            temp = (i - ylo) * ((float_t)-h) / (yhi - ylo) + (float_t)gy;

            d.drawLine(gx, temp, gx + w, temp, (i == 0) ? acolor : gcolor);

            d.setTextColor(tcolor, bcolor);

            if (digits)
            {
                d.drawFloat(i, digits, gx - 36, temp - 4, 1);
            }
            else
            {
                d.drawNumber(i, gx - 18, temp - 4, 1);
            }
        }

        // Draw x scale
        //***************************************************************************
        int32_t num = 0;
        for (i = xlo; i <= xhi; i += xinc)
        {
            // compute the transform
            temp = (i - xlo) * ((float_t)w) / (xhi - xlo) + (float_t)gx;

            d.drawLine(temp, gy, temp, gy - h, (i == 0) ? acolor : gcolor);

            // 0 - 12 by 2
            if (num < 10)
            {
                d.drawNumber(num, temp - 3, XNUMS_UL_Y, 1);
            }
            else
            {
                d.drawNumber(num, temp - 6, XNUMS_UL_Y, 1);
            }

            // increment by 1, 2 or 4 hours
            switch ((uint32_t)xhi)
            {
            case GR_6H:
                num += 1;
                break;
            case GR_24H:
                num += 4;
                break;
            case GR_12H:
            default:
                num += 2;
            }
        }

        // Now draw the labels
        //*****************************************************************************
        d.setTextColor(tcolor, bcolor);
        d.drawString(title, TITLE_UL_X, TITLE_UL_Y, 1);
    }

    // graph drawn, now plot the data
    //*****************************************************************************
    // The entire plotting code are these few lines...
    // Recall that ox and oy are initialized as static above
    x = (x - xlo) * ((float_t)w) / (xhi - xlo) + (float_t)gx;
    y = (y - ylo) * ((float_t)-h) / (yhi - ylo) + (float_t)gy;

    d.drawLine(ox + 1, oy, x + 1, y, pcolor);
    d.drawLine(ox, oy, x, y, pcolor);

    ox = x;
    oy = y;
}

//   Helper function for "graph()" -- draws the setpoint line
//   &d name of your display object
//   y = setpoint
//   gx = x graph location (lower left)
//   gy = y graph location (lower left)
//   w = width of graph
//   h = height of graph
//   xlo = lower bound of x axis
//   xhi = upper bound of x asis
//   ylo = lower bound of y axis
//   yhi = upper bound of y asis
//   tcolor = setpoint color
void drawSetpointLine(TFT_eSPI &d,
                      float_t y,
                      int32_t gx, int32_t gy,
                      int32_t w, int32_t h,
                      float_t xlo, float_t xhi,
                      float_t ylo, float_t yhi,
                      uint32_t tcolor)
{
    float_t ox, oy;
    float_t x;

    // X0 point
    x = xlo;
    ox = (x - xlo) * ((float_t)w) / (xhi - xlo) + (float_t)gx;
    oy = (y - ylo) * ((float_t)-h) / (yhi - ylo) + (float_t)gy;

    // X1 point
    x = xhi;
    x = (x - xlo) * ((float_t)w) / (xhi - xlo) + (float_t)gx;
    y = (y - ylo) * ((float_t)-h) / (yhi - ylo) + (float_t)gy;

    d.drawLine(ox, oy, x, y, tcolor);
    if (x < (XHI - 1))
    {
        d.drawLine(ox + 1, oy, x + 1, y, tcolor);
    }
}
