//
//   This function provides a Cartesian graph implementation.
//   Originally written by Kris Kasprzak:
//   https://github.com/KrisKasprzak/Graphing
//

#include <Arduino.h>
#include "LRThermostat.h"
#include "LRThermostat_menu.h"

void Graph(TFT_eSPI &d,
           float_t x, float_t y,
           int32_t gx, int32_t gy, int32_t w, int32_t h,
           float_t xlo, float_t xhi, float_t xinc,
           float_t ylo, float_t yhi, float_t yinc,
           String title, String xlabel, String ylabel,
           uint32_t gcolor, uint32_t acolor, uint32_t pcolor,
           uint32_t tcolor, uint32_t bcolor,
           boolean &redraw, uint32_t digits);

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

// Call with drawGrid == true. The coordinate system will only be drawn once.
void graphBaro(boolean drawGrid)
{
    uint32_t low = cbBaro[0];
    uint32_t high = low;

    // Scan data to get its range
    for (int32_t n = 1; n < cbBaro.size(); n++)
    {
        uint32_t tmp = cbBaro[n];

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
    // Note: these round up/downs are using integer math then floating for final division
    float_t ylo = ((float_t)(low / 100)) / 10;         // round down to nearest 0.100
    float_t yhi = ((float_t)((high + 99) / 100)) / 10; // round up to nearest 0.100

    tft.fillScreen(BLACK);

    for (int32_t x = 0; x < cbBaro.size(); x++)
    {
        float_t y = ((float_t)(cbBaro[x])) / 1000;

        Graph(tft,
              (float_t)x, y,             // data point
              40, 100,                   // lower left corner of graph
              110, 90,                   // width, height
              0, 72, 12,                 // xlow, xhi, xinc
              ylo, yhi, (yhi - ylo) / 5, // ylow, yhi, yinc
              "12 hr Baro Pres", "", "", // title, x-label, y-label
              DKBLUE,                    // grid line color
              DKBLUE,                    // axis lines color
              YELLOW,                    // plotted data color
              WHITE,                     // text color
              BLACK,                     // background color
              drawGrid,                  // redraw flag
              2);                        // digits
    }
}

// Call with drawGrid == true. The coordinate system will only be drawn once.
void graphHumidity(boolean drawGrid)
{
    uint32_t low = cbHumd[0];
    uint32_t high = low;

    // Scan data to get its range
    for (int32_t n = 1; n < cbHumd.size(); n++)
    {
        uint32_t tmp = cbHumd[n];

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
    // Note: these round up/downs are using integer math exclusively
    float_t ylo = low / 500 * 5;        // round down to nearest 5
    float_t yhi = (high + 499) / 500 * 5; // round up to nearest 5

    tft.fillScreen(BLACK);

    for (int32_t x = 0; x < cbHumd.size(); x++)
    {
        float_t y = ((float_t)cbHumd[x])/100;

        Graph(tft,
              (float_t)x, y,             // data point
              40, 100,                   // lower left corner of graph
              110, 90,                   // width, height
              0, 72, 12,                 // xlow, xhi, xinc
              ylo, yhi, (yhi - ylo) / 5, // ylow, yhi, yinc
              " 12 hr Humidity", "", "", // title, x-label, y-label
              DKBLUE,                    // grid line color
              DKBLUE,                    // axis lines color
              YELLOW,                    // plotted data color
              WHITE,                     // text color
              BLACK,                     // background color
              drawGrid,                  // redraw flag
              0);                        // digits
    }
}

// Call with drawGrid == true. The coordinate system will only be drawn once.
void graphTemperature(boolean drawGrid)
{
    uint32_t low = cbTemp[0];
    uint32_t high = low;

    // Scan data to get its range
    for (int32_t n = 1; n < cbTemp.size(); n++)
    {
        uint32_t tmp = cbTemp[n];

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
    // Note: these round up/downs are using integer math exclusively
    float_t ylo = low / 500 * 5;        // round down to nearest 5
    float_t yhi = (high + 499) / 500 * 5; // round up to nearest 5

    tft.fillScreen(BLACK);

    for (int32_t x = 0; x < cbTemp.size(); x++)
    {
        float_t y = ((float_t)cbTemp[x])/100;

        Graph(tft,
              (float_t)x, y,               // data point
              40, 100,                     // lower left corner of graph
              110, 90,                     // width, height
              0, 72, 12,                   // xlow, xhi, xinc
              ylo, yhi, (yhi - ylo) / 5,   // ylow, yhi, yinc
              "12 hr Temperature", "", "", // title, x-label, y-label
              DKBLUE,                      // grid line color
              DKBLUE,                      // axis lines color
              YELLOW,                      // plotted data color
              WHITE,                       // text color
              BLACK,                       // background color
              drawGrid,                    // redraw flag
              0);                          // digits
    }
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
void Graph(TFT_eSPI &d,
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
                d.drawFloat(i, digits, gx - 40, temp - 2, 1);
            }
            else
            {
                d.drawNumber(i, gx - 18, temp - 2, 1);
            }
        }

        // Draw x scale
        //***************************************************************************
        for (i = xlo; i <= xhi; i += xinc)
        {
            // compute the transform
            temp = (i - xlo) * ((float_t)w) / (xhi - xlo) + (float_t)gx;

            d.drawLine(temp, gy, temp, gy - h, (i == 0) ? acolor : gcolor);
        }

        // Now draw the labels
        //*****************************************************************************
        d.setTextColor(tcolor, bcolor);
        d.drawString(title, gx + 7, gy + 10, 1);

        d.setTextColor(acolor, bcolor);
        d.drawString(xlabel, gx, gy + 20, 1);

        d.setTextColor(acolor, bcolor);
        d.drawString(ylabel, gx - 30, gy - h - 10, 1);
    }

    // Graph drawn, now plot the data
    //*****************************************************************************
    // The entire plotting code are these few lines...
    // Tecall that ox and oy are initialized as static above
    x = (x - xlo) * ((float_t)w) / (xhi - xlo) + (float_t)gx;
    y = (y - ylo) * ((float_t)-h) / (yhi - ylo) + (float_t)gy;

    d.drawLine(ox + 1, oy, x + 1, y, pcolor);
    d.drawLine(ox, oy, x, y, pcolor);

    ox = x;
    oy = y;
}
