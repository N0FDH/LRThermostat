// Some of the following web server implemetation and HTML was derived from a project by David Bird, whose license is as follows:
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
** BASIC STATUS PAGE OUTPUT FORMAT (4x18 table)
**
*1*                    Temperature       Humidity       Pressure
*2*                        65*             34%         28.74 inHg
*3*
*4* current mode	      heat
*5* current state	       on
*6* fan state		      auto
*7*
*8*                        Heat            Cool         Dehumidify
*9* setpoint	           66*		       70*		       55%
*0* hysteresis(+/-)	     0.5/0.5*	     0.5/0.5*	       10/0%
*1* usage**		        18:32:04	     18:32:04	     18:32:04
*2* calibration           -6.0             -6.0             +10
*3* min run time			                               30 min
*4*
*5* **usage reset		3/4/22 18:32:04
*6* boot time		    3/4/22 18:32:04
*7* boot count		    24
*8* FW version		    1.0.0
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "LRThermostat.h"
#include "WifiCredentials.h" // Modify "WifiCredentials.h-template" with your credentials

String webpage = ""; // General purpose variable to hold HTML code for display
String sitetitle = "LR Thermostat";

// Wifi and web server stuff
AsyncWebServer server(80);
void WifiSetup();
void SetupWebpageHandlers();
void append_HTML_header(bool refreshMode);
void append_HTML_footer();
void Homepage();

//#########################################################################################
void wifiSetup()
{
    Serial.printf("Connecting to: %s\n", ssid);
    WiFi.disconnect();
    WiFi.mode(WIFI_STA); // switch off AP
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(100);
    }
    Serial.println("\nWiFi connected, IP = " + WiFi.localIP().toString());

    SetupWebpageHandlers();
    server.begin();

    Serial.println("Web server started.");
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

    append_HTML_header(20);
    webpage += "<h2>Smart Thermostat Status</h2><br>";
    //    webpage += "<div class='numberCircle'><span class=" + String((RelayState == "ON" ? "'on'>" : "'off'>")) + String(Temperature, 1) + "&deg;</span></div><br><br><br>";
    webpage += "<table class='centre'>";
    webpage += "<tr>";
    webpage += "<td>Temperature</td>";
    webpage += "<td>Humidity</td>";
    webpage += "<td>Target Temperature</td>";
    webpage += "<td>Thermostat Status</td>";
    webpage += "<td>Schedule Status</td>";
    webpage += "</tr>";
    webpage += "<tr>";
    webpage += "<td class='large'>" + String(curTemp, 1) + "&deg;</td>";
    webpage += "<td class='large'>" + String(curHumd, 0) + "%</td>";
    webpage += "<td class='large'>" + String((float)setPt, 1) + "&deg;</td>";
    webpage += "</tr>";
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
    if (refreshMode)
        webpage += "<meta http-equiv='refresh' content='5'>"; // 5-secs refresh time, test needed to prevent auto updates repeating some commands
                                                              //  webpage += "<script src=\"https://code.jquery.com/jquery-3.2.1.min.js\"></script>";
    webpage += "<style>";
    webpage += "body             {width:68em;margin-left:auto;margin-right:auto;font-family:Arial,Helvetica,sans-serif;font-size:14px;color:blue;background-color:#e1e1ff;text-align:center;}";
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
    webpage += ".numberCircle    {border-radius:50%;width:2.7em;height:2.7em;border:0.11em solid blue;padding:0.2em;color:blue;text-align:center;font-size:3em;";
    webpage += "                  display:inline-flex;justify-content:center;align-items:center;}";
    webpage += ".wifi            {padding:3px;position:relative;top:1em;left:0.36em;}";
    webpage += ".wifi, .wifi:before {display:inline-block;border:9px double transparent;border-top-color:currentColor;border-radius:50%;}";
    webpage += ".wifi:before     {content:'';width:0;height:0;}";
    webpage += "</style></head>";
    webpage += "<body>";
    webpage += "<div class='topnav'>";
    webpage += "<a href='/'>Status</a>";
    webpage += "<a href='graphs'>Graph</a>";
    webpage += "<a href='timer'>Schedule</a>";
    webpage += "<a href='setup'>Setup</a>";
    webpage += "<a href='help'>Help</a>";
    webpage += "<a href=''></a>";
    webpage += "<a href=''></a>";
    webpage += "<a href=''></a>";
    webpage += "<a href=''></a>";
    webpage += "<div class='wifi'/></div><span>" + WiFiSignal() + "</span>";
    webpage += "</div><br>";
}
//#########################################################################################
void append_HTML_footer()
{
    webpage += "<footer>";
    webpage += "</footer>";
    webpage += "</body></html>";
}
