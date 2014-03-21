#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <EcoSPI.h>
#include <EcoSensors.h>
#include "pins_arduino.h"
#include <DS1302.h>

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)
#define DAYS_PER_WEEK (7UL)
#define SECS_PER_WEEK (SECS_PER_DAY * DAYS_PER_WEEK)
#define SECS_PER_YEAR (SECS_PER_WEEK * 52UL)
#define SECS_YR_2000  (946684800UL) // the time at the start of y2k

// leap year calulator expects year argument as years offset from 1970
#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )

// PINS
#define BRAIN_SS_PIN            2
#define LED_RED_PIN             3
// WiFi 4
#define LED_GREEN_PIN           5
#define LED_BLUE_PIN            6
// WiFi 7
#define ROBOT_SS_PIN            8
#define ROBOT_READY_PIN         9
// Wifi 10

#define BRAIN_READY_PIN        A0
#define ULTRASONIC_SS_PIN      A1
#define ENVIRONMENT_SS_PIN     A2
#define DS1302_SCLK_PIN        A3    // Arduino pin for the Serial Clock
#define DS1302_IO_PIN          A4    // Arduino pin for the Data I/O
#define DS1302_CE_PIN          A5    // Arduino pin for the Chip Enable


static  const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // API starts months from 1, this array starts from 0
DS1302 rtc(DS1302_SCLK_PIN, DS1302_IO_PIN, DS1302_CE_PIN); // Real Time Clock

EcoSPI brain_espi(BRAIN_SS_PIN, DEVICE_BRAIN);
EcoSPI robot_espi(ROBOT_SS_PIN, DEVICE_ROBOT);
EcoSPI ultrasonic_espi(ULTRASONIC_SS_PIN, DEVICE_ULTRASONIC);
EcoSPI environment_espi(ENVIRONMENT_SS_PIN, DEVICE_ENVIRONMENT);
 
String timestamp;
 
/* WiFi Code - should be it's own class */
char ssid[] = "MySSID"; //  your network SSID (name) 
char pass[] = "MyPassword";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

IPAddress server(192, 168, 1, 1);
unsigned int port = 5432;

WiFiUDP Udp;

void setup (void)
{
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  
  pinMode(BRAIN_READY_PIN, INPUT);
  pinMode(ROBOT_READY_PIN, INPUT);

  // Set the clock to run-mode, and disable the write protection
  rtc.halt(false);
//  rtc.writeProtect(false);

  // The following lines can be commented out to use the values already stored in the DS1302
//  rtc.setDOW(FRIDAY);        // Set Day-of-Week to FRIDAY
//  rtc.setTime(9, 56, 50);     // Set the time to 12:00:00 (24hr format)
//  rtc.setDate(21, 2, 2014);   // Set the date to August 6th, 2010

  // Setup Serial connection  
  Serial.begin (115200);
  Serial.println ();

  delay(10);

  analogWrite(LED_RED_PIN, 255);
  analogWrite(LED_GREEN_PIN, 127);
  analogWrite(LED_BLUE_PIN, 80);
  
  delay(10000);
  
  analogWrite(LED_RED_PIN, 138);
  analogWrite(LED_GREEN_PIN, 43);
  analogWrite(LED_BLUE_PIN, 226);
  
  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();

  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV8);
}  // end of setup

void loop (void)
{
  Serial.println("-------------------------------------------------");
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present"); 
  } else {
    if (status != WL_CONNECTED) {
      while (status != WL_CONNECTED) { 
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:    
        status = WiFi.begin(ssid, pass);
        
        // wait 1 seconds for connection:
        delay(1000);
      } 
      
      Serial.println("Connected: ");
      Serial.println();      
      printWifiStatus();
    }
  }

  // Robot tells us when it has finished moving
  if (robot_espi.isConnected()) {
    while (digitalRead(ROBOT_READY_PIN) == HIGH) { 
      // wait for a pin to go high/low so we know the brain has worked it's stuff out
      delay (10);
    }
    Serial.println("Robot is ready!");
  }
  
  EcoSensors es;

  es.rtc = getEpochTime(rtc.getTime());  
  timestamp = (String) es.rtc;
  Serial.println("Timestamp: " + timestamp);
  
  if (robot_espi.getReadings(es) && WiFi.status() != WL_NO_SHIELD) {
    Serial.println("Sending robot data");
    _sendReading("compass", String(es.compassBearing),   "d", timestamp);
  }

  if (ultrasonic_espi.getReadings(es) && WiFi.status() != WL_NO_SHIELD) {
    Serial.println("Sending ultrasonic data");
    _sendReading("range1", String(es.ultrasonic0),   "cm", timestamp);
    _sendReading("range2", String(es.ultrasonic45),  "cm", timestamp);
    _sendReading("range3", String(es.ultrasonic90),  "cm", timestamp);
    _sendReading("range4", String(es.ultrasonic135), "cm", timestamp);
    _sendReading("range5", String(es.ultrasonic180), "cm", timestamp);
    _sendReading("range6", String(es.ultrasonic225), "cm", timestamp);
    _sendReading("range7", String(es.ultrasonic270), "cm", timestamp);
    _sendReading("range8", String(es.ultrasonic315), "cm", timestamp);
  }
  
  if (environment_espi.getReadings(es) && WiFi.status() != WL_NO_SHIELD) {
    Serial.println("Sending environment data");
    _sendReading("lightLeft", String(es.lightLeft), "Ω", timestamp);
    _sendReading("lightRight", String(es.lightRight), "Ω", timestamp);
    _sendReading("carbon", String(es.COppm), "ppm", timestamp);
  }

  if (es.temperatureLeft < 20) {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 255);
  } else if (es.temperatureLeft < 30) {
    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 255);
    analogWrite(LED_BLUE_PIN, 0);    
  } else {
    analogWrite(LED_RED_PIN, 255);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);
  }

  if (brain_espi.isConnected()) {
    // Send EcoSensor readings to the brain
    brain_espi.sendReadings(es);
    
    SPI_BRAIN_COMMAND calc;
    
    calc.command = BRAIN_CALCULATE;
    calc.size = 0;
    
    brain_espi.sendCommand(calc);
  
    // Brain can calculate if it's ready to do something by setting BRAIN_READY pin to low
    while (digitalRead(BRAIN_READY_PIN) == HIGH) { 
      // wait for a pin to go high/low so we know the brain has worked it's stuff out
      delay (10);
    }
    
    Serial.println("Brain is ready!");
    
    SPI_BRAIN_COMMAND cmd;
    if (brain_espi.getCommand(cmd)) {    
      robot_espi.sendCommand(cmd);
    }
  }
}  // end of loop

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void _sendReading(String sensor, String value, String unit, String timestamp) {
  String json = "{\"sensor\":\""+ sensor + "\",\"value\":\""+ value +"\",\"unit\":\"" + unit + "\",\"date\":\""+ timestamp + "\"}| ";
  
  int len = json.length();
  char buf[len];
  uint8_t buf2[len];
  
  json.toCharArray(buf, len);
  
  for (int i=0; i<=len; i++) { 
    buf2[i] = buf[i];
  }
  
  Udp.beginPacket(server, port);
  Udp.write(buf2, len);
  Udp.endPacket();
  
  delay(100);
}

// Modified from Time.h to work with time from DS1302 library
uint32_t getEpochTime(Time tm){   
  int i;
  uint32_t seconds;
  int year = tm.year - 1970;

  // seconds from 1970 till 1 jan 00:00:00 of the given year
  seconds= year*(SECS_PER_DAY * 365);
  for (i = 0; i < year; i++) {
    if (LEAP_YEAR(i)) {
      seconds +=  SECS_PER_DAY;   // add extra days for leap years
    }
  }
  
  // add days for this year, months start from 1
  for (i = 1; i < tm.mon; i++) {
    if ( (i == 2) && LEAP_YEAR(year)) { 
      seconds += SECS_PER_DAY * 29;
    } else {
      seconds += SECS_PER_DAY * monthDays[i-1];  //monthDay array starts from 0
    }
  }
  seconds+= (tm.date-1) * SECS_PER_DAY;
  seconds+= tm.hour * SECS_PER_HOUR;
  seconds+= tm.min * SECS_PER_MIN;
  seconds+= tm.sec;
  return seconds; 
}
