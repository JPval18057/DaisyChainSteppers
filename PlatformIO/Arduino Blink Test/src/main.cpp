/* PLATFORMIO TEST CODE
* AUTHOR: JUAN PABLO VALENZUELA
* DESCRIPTION: TURN ON AND OFF A BLUE LED
*/
#include <Arduino.h>

//Pin 13 is the Built in LED
#define LED 10
//We use pin 10 to turn on an LED

//Variables
int ledstate = LOW;
long current,previous; //time variables
long interval = 500; //Time period of LED

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  current = millis();
  if ((current - previous)>=interval){
    previous = current;
    ledstate = !ledstate;
    digitalWrite(LED,ledstate);
  }

}