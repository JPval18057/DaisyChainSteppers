/*************************************************************

  This is a simple demo of sending and receiving some data.
  Be sure to check out other examples!
 *************************************************************/

//Macros
// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID           "TMPLJTR6833E"
#define BLYNK_DEVICE_NAME           "Quickstart Device"
#define BLYNK_AUTH_TOKEN            "2-al7tyzbydJCPHsRq26UWLLON3_UuNU"

//Define usable GPIOS
#define LED 2 //Built in LED
#define LED1 15
#define LED2 0
#define LED3 16
#define LED4 5

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial

//Libraries
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

//Objects
BlynkTimer timer;

//Variables

char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "ARRIS-B2E1";
char pass[] = "50A5DC03B2E1";

//Millis and Micros variables
long m_current,u_current,m_prev,u_prev;
const long interval = 1000;
bool toggle = HIGH;

//Functions
// This function is called every time the RTC changes
BLYNK_WRITE(InternalPinRTC)
{
  // Set incoming value from pin V0 to a variable
  long RTCTime = param.asLong();
  // RTCTime = "\n" + RTCTime + "\n";
  // Update state
  // Serial.print("Unix time: ");
  // Serial.print(RTCTime); //in Unix format
}

//This function is called every time the virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  int ledstate = param.asInt();
  digitalWrite(LED1, ledstate); 
  digitalWrite(LED2, ledstate); 
  digitalWrite(LED3, ledstate); 
  digitalWrite(LED4, ledstate); 
}
// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.

  //request current local time for device
  Blynk.sendInternal("rtc", "sync"); 
  toggle = !toggle;
  digitalWrite(LED, toggle); 


}

void setup()
{
  // Debug console
  Serial.begin(115200);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
  //IO Configuration
  pinMode(LED, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

}

void loop()
{
  Blynk.run();
  timer.run();

  // This is a simple non blocking interval function
  if ((millis() - m_prev)>=interval)
  {
    m_prev = millis();
       
  }

}

