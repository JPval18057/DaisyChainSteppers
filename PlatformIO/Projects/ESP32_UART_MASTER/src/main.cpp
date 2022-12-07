//***********************************************************************************************
/*
Title: Daisy-Chain Master Code
Author: Juan Pablo Valenzuela
Date: 10/9/2022
Description:
This code controls the master microcontroller, in this case it is an ESP32 Dev Module.
This has all the definitions
*/
//***********************************************************************************************

//Macros
#define LED 2 //Built in LED
#define LED1 5 //Works, is the CSS of SPI-V
#define LED2 25
#define LED3 26
#define CommsBaudrate 115200

//Pins 36, 39, 34 and 35 are input only
//TX2 is GPIO 17 and RX2 is GPIO 16

//Libraries

#include <Arduino.h>
#include <ArduinoJson.h>
//#include <EEPROM.h> //In case we need to save something to memory


//Objects

//Variables

//Debug timing
long m_current,m_previous,u_current,u_previous;
const long m_interval = 100;
const long u_interval = 100;
uint8_t toggle = LOW;

//Functions

//***********************************************************************************************
//Main Code
//***********************************************************************************************

void setup() {
  // put your setup code here, to run once:
  //Comms config
  Serial.begin(CommsBaudrate);   //Default settings
  Serial2.begin(CommsBaudrate);  //Default settings
  /*
  * Serial 0 is to communicate with the PC
  * Serial 1 is to communicate with the Slaves
  */
  pinMode(LED, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  m_previous = millis();
  u_previous = micros();


}

void loop() {
  // put your main code here, to run repeatedly:
  m_current = millis();
  if ((m_current - m_previous)>=m_interval)
  {
    m_previous = m_current;
    toggle = !toggle;
    digitalWrite(LED, toggle);
    digitalWrite(LED1, toggle);
    digitalWrite(LED2, toggle);
    digitalWrite(LED3, toggle);
  }
}

//***********************************************************************************************
/*
 *   Serial Communication Readings
*/
//***********************************************************************************************



//***********************************************************************************************
/*
 *   Program Functions
*/
//***********************************************************************************************

