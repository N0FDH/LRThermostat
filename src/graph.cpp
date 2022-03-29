/*
This program provides cartesian type graph function
Revisions
rev     date        author      description
1       12-24-2015  kasprzak    initial creation

     https://github.com/KrisKasprzak/Graphing
*/

#include <Arduino.h>
#include "LRThermostat.h"
#include "LRThermostat_menu.h"

void Graph(TFT_eSPI &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw);

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

double ox, oy;

// Call with drawAll == true. The the coordinate system will only brawn once.
void graphBaro(boolean drawAll)
{

  tft.fillScreen(BLACK);

  int32_t x;
  double y;

  uint32_t low = oldBaro[0];
  uint32_t high = oldBaro[0];

  for (int32_t n = 1; n < 72; n++)
  {
    if (oldBaro[n] < low)
    {
      low = oldBaro[n];
    }
    else if (oldBaro[n] > high)
    {
      high = oldBaro[n];
    }
  }

  double ylo = ((double)(low / 100)) / 10;         // round down to nearest 0.100
  double yhi = ((double)((high + 99) / 100)) / 10; // round up to nearest 0.100

  //  Serial.printf("hi %u, low %u, yhi %f, ylo %f\n", high, low, yhi, ylo);

  for (x = 0; x < BARO_CNT; x++)
  {
    y = ((double)(oldBaro[(BARO_CNT - 1) - x])) / 1000;

    Graph(tft,
          (double)x, y,              // data point
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
          drawAll);                  // redraw flag
  }
}

/*
  function to draw a cartesian coordinate system and plot whatever data you want
  just pass x and y and the graph will be drawn
  huge arguement list
  &d name of your display object
  x = x data point
  y = y datapont
  gx = x graph location (lower left)
  gy = y graph location (lower left)
  w = width of graph
  h = height of graph
  xlo = lower bound of x axis
  xhi = upper bound of x asis
  xinc = division of x axis (distance not count)
  ylo = lower bound of y axis
  yhi = upper bound of y asis
  yinc = division of y axis (distance not count)
  title = title of graph
  xlabel = x asis label
  ylabel = y asis label
  gcolor = graph line colors
  acolor = axi ine colors
  pcolor = color of your plotted data
  tcolor = text color
  bcolor = background color
  &redraw = flag to redraw graph on fist call only
*/

void Graph(TFT_eSPI &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw)
{
  //  double ydiv, xdiv;
  // initialize old x and old y in order to draw the first point of the graph
  // but save the transformed value
  // note my transform funcition is the same as the map function, except the map uses long and we need doubles
  // static double ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
  // static double oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  double i;
  double temp;

  if (redraw == true)
  {
    redraw = false;
    ox = (x - xlo) * (w) / (xhi - xlo) + gx;
    oy = (y - ylo) * (-h) / (yhi - ylo) + gy;

    // draw y scale
    //*****************************************************************************
    for (i = ylo; i <= (yhi + .0001); i += yinc) // add in a tiny amount to be sure to get last line
    {
      // compute the transform
      temp = (i - ylo) * (-h) / (yhi - ylo) + gy;

      d.drawLine(gx, temp, gx + w, temp, (i == 0) ? acolor : gcolor);

      d.setTextColor(tcolor, bcolor);
      d.drawFloat(i, 2, gx - 40, temp - 2, 1);
    }

    // draw x scale
    //*****************************************************************************
    for (i = xlo; i <= xhi; i += xinc)
    {
      // compute the transform
      temp = (i - xlo) * (w) / (xhi - xlo) + gx;

      d.drawLine(temp, gy, temp, gy - h, (i == 0) ? acolor : gcolor);

      // d.setTextColor(tcolor, bcolor);
      // d.drawFloat(i, 0, temp, gy + 10, 1);
      // d.drawNumber(num, temp - 3, gy + 10, 1);
    }

    // now draw the labels
    //*****************************************************************************
    d.setTextColor(tcolor, bcolor);
    // d.drawString(title, gx, gy - h - 30, 1);
    d.drawString(title, gx + 7, gy + 10, 1);

    d.setTextColor(acolor, bcolor);
    d.drawString(xlabel, gx, gy + 20, 1);

    d.setTextColor(acolor, bcolor);
    d.drawString(ylabel, gx - 30, gy - h - 10, 1);
  }

  // graph drawn, now plot the data
  //*****************************************************************************
  //  the entire plotting code are these few lines...
  //  recall that ox and oy are initialized as static above
  x = (x - xlo) * (w) / (xhi - xlo) + gx;
  y = (y - ylo) * (-h) / (yhi - ylo) + gy;

  d.drawLine(ox + 1, oy, x + 1, y, pcolor);
  d.drawLine(ox, oy, x, y, pcolor);
  //  d.drawLine(ox, oy + 1, x, y + 1, pcolor);
  // d.drawWideLine(ox, oy + 1, x, y + 1, 1, pcolor);
  ox = x;
  oy = y;
}
