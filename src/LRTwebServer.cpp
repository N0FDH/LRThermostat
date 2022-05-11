// Some of the following web server implemetation and HTML was derived from a project by David Bird,
// https://github.com/G6EJD/ESP-SMART-Thermostat, whose license is as follows:
/*
  This software, the ideas and concepts is Copyright (c) David Bird 2020
  All rights to this software are reserved.
  It is prohibited to redistribute or reproduce of any part or all of the software contents in any form other than the following:
  1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
  2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author David Bird
     as the source of the material.
  3. You may not, except with my express written permission, distribute or commercially exploit the content.
  4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.
  5. You MUST include all of this copyright and permission notice ('as annotated') and this shall be included in all copies or
     substantial portions of the software and where the software use is visible to an end-user.
  THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT.
  FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  CONTRACT, TORT OR OTHERWISE, ARISING FROM, OR OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  See more at http://dsbird.org.uk
*/

/*
** BASIC STATUS PAGE OUTPUT FORMAT (4x23 table)
**
*1*                    Temperature       Humidity       Pressure
*2*                        65*             34%         28.74 inHg
*3*
*4* calibration           -6.0            +5.0             +1.4
*5* current mode	      heat
*6* current state	       on
*7* fan state		      auto
*8*
*9*                        Heat            Cool         Dehumidify
*0* setpoint	           66*		       70*		       55%
*1* hysteresis(+/-)	     0.5/0.5*	     0.5/0.5*	       10/0%
*2* usage**		        18:32:04	     18:32:04	     18:32:04
*3* min run time			                               30 min
*4*
*5* **usage reset		3/4/22 18:32:04
*6* Uptime  		    56:18:32:04
*7* boot count		    24
*8* FW version		    1.0.0
*9* SSID                MyCharterWhateverTheHell
*0* IP                  192.168.0.132
*1* MAC                 01:23:45:67:89:AB
*2* Signal              71%
*3* BME280 resets       37
*4*
*/

#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "LRThermostat.h"

String webpage = ""; // General purpose variable to hold HTML code for display
String sitetitle = "LR Thermostat";

// Wifi and web server stuff
AsyncWebServer server(80);

void SetupWebpageHandlers();
void append_HTML_header(bool refreshMode);
void append_HTML_footer();
void Homepage();

//#########################################################################################
void serverSetup()
{
    server.end();

    SetupWebpageHandlers();

    // Start OTA server
    AsyncElegantOTA.begin(&server);

    // Start web server
    server.begin();

    Serial.printf("webserver up\n");
}

//#########################################################################################
// https://www.intuitibits.com/2016/03/23/dbm-to-percent-conversion/
String WiFiSignal(void)
{
    const unsigned char dBm2Percent[74] =
        {
            100, 99, 99, 99, 98, 98, 98, 97, 97, 96, // -20 .. -29
            96, 95, 95, 94, 93, 93, 92, 91, 90, 90,  // -30 .. -39
            89, 88, 87, 86, 85, 84, 83, 82, 81, 80,  // -40 .. -49
            79, 78, 76, 75, 74, 73, 71, 70, 69, 67,  // -50 .. -59
            66, 64, 63, 61, 60, 58, 56, 55, 53, 51,  // -60 .. -69
            50, 48, 46, 44, 42, 40, 38, 36, 34, 32,  // -70 .. -79
            30, 28, 26, 24, 22, 20, 17, 15, 13, 10,  // -80 .. -89
            8, 6, 3, 1                               // -90 .. -93
        };

    int sig = (int)WiFi.RSSI();
    int percent = 0; // no signal if not modified below

    if (sig > -20)
    {
        percent = 100;
    }
    else if (sig >= -93)
    {
        percent = dBm2Percent[(-20) - sig];
    }
    else if (sig >= -100)
    {
        percent = 1;
    }
    return String(percent) + "%";
}

//#########################################################################################
void SetupWebpageHandlers()
{
    // Set handler for '/'
    server.on("/", HTTP_GET,
              [](AsyncWebServerRequest *request)
              { request->redirect("/homepage"); });

    // Set handler for '/homepage'
    server.on("/homepage", HTTP_GET,
              [](AsyncWebServerRequest *request)
              { Homepage(); request->send(200, "text/html", webpage); });

    // Set handler for '/graphs'
    server.on("/graphs", HTTP_GET,
              [](AsyncWebServerRequest *request)
              { Homepage(); request->send(200, "text/html", webpage); });
}

//#########################################################################################
void Homepage()
{
    uint16_t setPt = 0;
    switch (mode)
    {
    case HEAT:
        setPt = loc.heatSetPt;
        break;

    case COOL:
        setPt = loc.acSetPt;
        break;

    case DEHUMIDIFY:
        setPt = loc.dhSetPt;
        break;

    case NO_MODE:
    default:
        break;
    }

    // get rid of compiler warning
    setPt = setPt;

    append_HTML_header(20);
    webpage += "<h2>LRThermostat Status</h2><br>";
    //------------------------------------
    webpage += "<table class='centre'>";
    //------------------------------------
    webpage += "<tr>";
    webpage += "<td>Temperature</td>";
    webpage += "<td>Humidity</td>";
    webpage += "<td>Barometric Pres.</td>";
    webpage += "<td>Mode</td>";
    webpage += "<td>State</td>";
    webpage += "<td>Fan</td>";
    webpage += "</tr>";
    //------------------------------------
    webpage += "<tr>";
    webpage += "<td class='medium'>" + String(curTemp, 0) + "&deg;</td>";
    webpage += "<td class='medium'>" + String(curHumd, 0) + "%</td>";
    webpage += "<td class='medium'>" + String(curBaro, 2) + " in</td>";
    //    webpage += "<td class='large'>" + String((float)setPt, 1) + "&deg;</td>";
    webpage += "<td class='medium'>"
               "Heat"
               "</td>";
    webpage += "<td class='medium'>"
               "On"
               "</td>";
    webpage += "<td class='medium'>"
               "Auto"
               "</td>";
    webpage += "</tr>";
    //------------------------------------
    webpage += "</table>";
    webpage += "<br>";
    append_HTML_footer();
}

//#########################################################################################
void append_HTML_header(bool refreshMode)
{
    webpage = "<!DOCTYPE html><html lang='en'>";
    webpage += "<head>";
    webpage += "<title>" + sitetitle + "</title>";
    webpage += "<meta charset='UTF-8'>";
    webpage += "<meta http-equiv='refresh' content='60'>";
    webpage += "<style>";
    webpage += "body             {width:30em;margin-left:auto;margin-right:auto;font-family:Arial,Helvetica,sans-serif;font-size:14px;color:blue;background-color:white;text-align:center;}";
    webpage += ".centre          {margin-left:auto;margin-right:auto;}";
    webpage += "h2               {margin-top:0.3em;margin-bottom:0.3em;font-size:1.4em;}";
    webpage += "h3               {margin-top:0.3em;margin-bottom:0.3em;font-size:1.2em;}";
    webpage += "h4               {margin-top:0.3em;margin-bottom:0.3em;font-size:0.8em;}";
    webpage += ".on              {color: red;}";
    webpage += ".off             {color: limegreen;}";
    webpage += ".topnav          {overflow: hidden;background-color:lightcyan;}";
    webpage += ".topnav a        {float:left;color:blue;text-align:center;padding:1em 1.14em;text-decoration:none;font-size:1.3em;}";
    webpage += ".topnav a:hover  {background-color:deepskyblue;color:white;}";
    webpage += ".topnav a.active {background-color:lightblue;color:blue;}";
    webpage += "table tr, td     {padding:0.2em 0.5em 0.2em 0.5em;font-size:1.0em;font-family:Arial,Helvetica,sans-serif;}";
    webpage += "col:first-child  {background:lightcyan}col:nth-child(2){background:#CCC}col:nth-child(8){background:#CCC}";
    webpage += "tr:first-child   {background:lightcyan}";
    webpage += ".large           {font-size:1.8em;padding:0;margin:0}";
    webpage += ".medium          {font-size:1.4em;padding:0;margin:0}";
    webpage += ".ps              {font-size:0.7em;padding:0;margin:0}";
    webpage += "#outer           {width:100%;display:flex;justify-content:center;}";
    webpage += "footer           {padding:0.08em;background-color:cyan;font-size:1.1em;}";
    webpage += "</style></head>";
    webpage += "<body>";
    webpage += "<div class='topnav'>";
    webpage += "<a href='/'>Status</a>";
    webpage += "<a href='graphs'>Graphs</a>";
    webpage += "<a href='sysInfo'>System Info</a>";
    webpage += "</div><br>";
}
//#########################################################################################
void append_HTML_footer()
{
    webpage += "<footer>";
    webpage += "</footer>";
    webpage += "</body></html>";
}

#if 0
//#########################################################################################
void Graphs()
{
    append_HTML_header(300);
    webpage += "<h2>Thermostat Readings</h2>";
    webpage += "<script type='text/javascript' src='https://www.gstatic.com/charts/loader.js'></script>";
    webpage += "<script type='text/javascript'>";
    webpage += "google.charts.load('current', {'packages':['corechart']});";
    webpage += "google.charts.setOnLoadCallback(drawGraphT1);"; // Pre-load function names for Temperature graphs
    webpage += "google.charts.setOnLoadCallback(drawGraphH1);"; // Pre-load function names for Humidity graphs
    AddGraph(1, "GraphT", "Temperature", "TS", "Â°C", "red", "chart_div");
    AddGraph(1, "GraphH", "Humidity", "HS", "%", "blue", "chart_div");
    webpage += "</script>";
    webpage += "<div id='outer'>";
    webpage += "<table>";
    webpage += "<tr>";
    webpage += "  <td><div id='chart_divTS1' style='width:50%'></div></td>";
    webpage += "  <td><div id='chart_divHS1' style='width:50%'></div></td>";
    webpage += "</tr>";
    webpage += "</table>";
    webpage += "<br>";
    webpage += "</div>";
    append_HTML_footer();
}
//#########################################################################################
String PreLoadChartData(byte Channel, String Type)
{
    byte r = 0;
    String Data = "";
    do
    {
        if (Type == "Temperature")
        {
            Data += "[" + String(r) + "," + String(sensordata[Channel][r].Temp, 1) + "," + String(TargetTemp, 1) + "],";
        }
        else
        {
            Data += "[" + String(r) + "," + String(sensordata[Channel][r].Humi) + "],";
        }
        r++;
    } while (r < SensorReadings);
    Data += "]";
    return Data;
}
//#########################################################################################
void AddGraph(byte Channel, String Type, String Title, String GraphType, String Units, String Colour, String Div)
{
    String Data = PreLoadChartData(Channel, Title);
    webpage += "function draw" + Type + String(Channel) + "() {";
    if (Type == "GraphT")
    {
        webpage += " var data = google.visualization.arrayToDataTable(" + String("[['Hour', 'Rm TÂ°', 'Tgt TÂ°'],") + Data + ");";
    }
    else
        webpage += " var data = google.visualization.arrayToDataTable(" + String("[['Hour', 'RH %'],") + Data + ");";
    webpage += " var options = {";
    webpage += "  title: '" + Title + "',";
    webpage += "  titleFontSize: 14,";
    webpage += "  backgroundColor: '" + backgrndColour + "',";
    webpage += "  legendTextStyle: { color: '" + legendColour + "' },";
    webpage += "  titleTextStyle:  { color: '" + titleColour + "' },";
    webpage += "  hAxis: {color: '#FFF'},";
    webpage += "  vAxis: {color: '#FFF', title: '" + Units + "'},";
    webpage += "  curveType: 'function',";
    webpage += "  pointSize: 1,";
    webpage += "  lineWidth: 1,";
    webpage += "  width:  450,";
    webpage += "  height: 280,";
    webpage += "  colors:['" + Colour + (Type == "GraphT" ? "', 'orange" : "") + "'],";
    webpage += "  legend: { position: 'right' }";
    webpage += " };";
    webpage += " var chart = new google.visualization.LineChart(document.getElementById('" + Div + GraphType + String(Channel) + "'));";
    webpage += "  chart.draw(data, options);";
    webpage += " };";
}
//#########################################################################################
#endif
