#include <Arduino.h>
#include <ArduinoJson.h>


#define BaudRate 115200
#define LED 10
#define BUFFER_SIZE 64

//JSON DOCUMENT
//Document to store incoming information
StaticJsonDocument<BUFFER_SIZE> Inputdoc;
//Document to store outgoing information
StaticJsonDocument<BUFFER_SIZE> Outputdoc;

//Variables
long u_current,u_previous,m_current,m_previous,u_interval; 
long interval=1000;
bool ledstate=false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
  {
    //Wait for the module
  };
  pinMode(LED,OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  m_current = millis();
  if ((m_current-m_previous)>=interval)
  {
    m_previous = m_current;
    ledstate =!ledstate;
    u_previous = micros();
    Serial.print("\nUART Test\n");
    u_current = micros();
    u_interval = u_current - u_previous;
    Serial.print("\nElapsed Time: ");
    Serial.print(u_interval);
  }
  digitalWrite(LED,ledstate);
}