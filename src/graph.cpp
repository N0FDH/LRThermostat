/*
This program provides cartesian type graph function
Revisions
rev     date        author      description
1       12-24-2015  kasprzak    initial creation
*/

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

#define ADJ_PIN A0

double a1,
    b1, c1, d1, r2, r1, vo, tempC, tempF, tempK;

//  Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC);

// this is the only external variable used by the graph
// it's a flat to draw the coordinate system only on the first pass
boolean display1 = true;
boolean display2 = true;
boolean display3 = true;
boolean display4 = true;
boolean display5 = true;
boolean display6 = true;
boolean display7 = true;
boolean display8 = true;
boolean display9 = true;
double ox, oy;

void graphit()
{

    tft.fillScreen(BLACK);

    a1 = 3.354016E-03;
    b1 = 2.569850E-04;
    c1 = 2.620131E-06;
    d1 = 6.383091E-08;

    int32_t x;
    double y;
    int16_t data[72] =
        {30025, 30034, 30041, 30044, 30040, 30019, 29995, 29990,
         29982, 29971, 29972, 29975, 29983, 29979, 29981, 29981,
         29978, 30014, 30009, 30010, 30000, 30000, 29995, 29993,
         29987, 29987, 29991, 29996, 30005, 30014, 30017, 30022,
         30021, 30031, 30025, 30050, 30071, 30083, 30092, 30100,
         30055, 30020, 29982, 29971, 29972, 29975, 29983, 29988,
         29990, 29995, 29999, 30014, 30009, 30010, 30000, 30000,
         29995, 29993, 29987, 29987, 29991, 29980, 29970, 29960,
         29940, 29931, 29921, 29900, 29921, 29934, 29950, 29950};

    uint32_t low = data[0];
    uint32_t high = data[0];

    for (int32_t n = 1; n < 72; n++)
    {
        if (data[n] < low)
        {
            low = data[n];
        }
        else if (data[n] > high)
        {
            high = data[n];
        }
    }

    double ylo = ((double)(low/100))/10; // round down to nearest 0.100
    double yhi = ((double)((high+99)/100))/10; // round up to nearest 0.100

    Serial.printf("hi %u, low %u, yhi %f, ylo %f\n", high, low, yhi, ylo);

    for (x = 0; x < 72; x++)
    {
        y = ((double)(data[x])) / 1000;
        //                   gx   gy   w    h   xl  xh xi  yl  yh yi
        //  Graph(tft, x, y, 60, 290, 390, 260, 0, 6.5, 1, -1, 1, .25, "Sin Function", "x", "sin(x)",

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
              display1);                 // redraw flag
    }

    delay(100000);
}

#if 0
  for (x = 0; x <= 6.3; x += .1)
  {

    y = sin(x);
    //                   gx   gy   w    h   xl  xh xi  yl  hi yi
    //  Graph(tft, x, y, 60, 290, 390, 260, 0, 6.5, 1, -1, 1, .25, "Sin Function", "x", "sin(x)",
    Graph(tft, x, y, 40, 88, 110, 80, 0, 6.5, 1, -1, 1, .25, "Sin Function", "x", "sin(x)", DKBLUE, RED, YELLOW, WHITE, BLACK, display1);
  }

  delay(1000);

  tft.fillScreen(BLACK);
  for (x = 0; x <= 6.3; x += .1) {

    y = sin(x);
    Graph(tft, x, y, 100, 280, 100, 240, 0, 6.5, 3.25, -1, 1, .25, "Sin Function", "x", "sin(x)", GREY, GREEN, RED, YELLOW, BLACK, display9);

  }

    delay(1000);

  tft.fillScreen(BLACK);
  for (x = 0; x <= 25.2; x += .1) {

    y = sin(x);
    Graph(tft, x, y, 50, 190, 400, 60, 0, 25, 5, -1, 1, .5, "Sin Function", "x", "sin(x)", DKYELLOW, YELLOW, GREEN, WHITE, BLACK, display8);

  }

  delay(1000);

  tft.fillScreen(BLACK);
  for (x = 0.001; x <= 10; x += .1) {

    y = log(x);
    Graph(tft, x, y, 50, 240, 300, 180, 0, 10, 1, -10, 5, 1, "Natural Log Function", "x", "ln(x)", BLUE, RED, WHITE, WHITE, BLACK, display2);

  }

  

  delay(1000);
  tft.fillScreen(BLACK);

  for (x = 0; x <= 10; x += 1) {

    y = x * x;
    Graph(tft, x, y, 50, 290, 390, 260, 0, 10, 1, 0, 100, 10, "Square Function", "x", "x^2", DKRED, RED, YELLOW, WHITE, BLACK, display3);

  }

  delay(1000);
  tft.fillScreen(BLACK);

  for (x = 0.00; x <= 20; x += .01) {

    y = ((sin(x)) * x + cos(x)) - log(x);
    Graph(tft, x, y, 50, 290, 390, 260, 0, 20, 1, -20, 20, 5, "Weird Function", "x", " y = sin(x) + cos(x) - log(x)", ORANGE, YELLOW, CYAN, WHITE, BLACK, display4);

  }

  delay(1000);
  tft.fillScreen(BLACK);
  tft.setRotation(2);
  for (x = 0; x <= 12.6; x += .1) {

    y = sin(x);
    Graph(tft, x, y, 50, 250, 150, 150, 0, 13, 3.5, -1, 1, 1, "Sin(x)", "x", "sin(x)", DKBLUE, RED, YELLOW, WHITE, BLACK, display5);

  }
  tft.setRotation(3);
  delay(1000);
  tft.fillScreen(WHITE);

  for (x = 0; x <= 6.3; x += .05) {

    y = cos(x);
    Graph(tft, x, y, 100, 250, 300, 200, 0, 6.5, 3.25, -1, 1, 1, "Cos Function", "x", "cos(x)", DKGREY, GREEN, BLUE, BLACK, WHITE, display6);

  }

  delay(1000);
  tft.fillScreen(BLACK);


  for (x = 0; x <= 60; x += 1) {
    vo = analogRead(ADJ_PIN) / 204.6;
    r1 = 9940;
    r2 = ( vo * r1) / (5 - vo);

    //equation from data sheet
    tempK = 1.0 / (a1 + (b1 * (log(r2 / 10000.0))) + (c1 * pow(log(r2 / 10000.0), 2)) + (d1 * pow(log(r2 / 10000.0), 3)));
    tempC  = ((tempK - 273.15) );
    y = tempF  = (tempC * 1.8000) + 32.00;

    Graph(tft, x, y, 50, 290, 390, 260, 0, 60, 10, 70, 90, 5, "Room Temperature", " Time [s]", "Temperature [deg F]", DKBLUE, RED, GREEN, WHITE, BLACK, display7);
    delay(250);
  }

 delay(1000);
  tft.fillScreen(BLACK);
}
#endif

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
    //d.drawWideLine(ox, oy + 1, x, y + 1, 1, pcolor);
    ox = x;
    oy = y;
}
