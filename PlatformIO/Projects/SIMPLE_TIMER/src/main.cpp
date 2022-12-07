// These define's must be placed at the beginning before #include "TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#define USE_TIMER_1     false

#if ( defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)  || \
        defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) ||    defined(ARDUINO_AVR_ETHERNET) || \
        defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT)   || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO)      || \
        defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) || \
        defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
        defined(ARDUINO_AVR_PROTRINKET3FTDI) )
  #define USE_TIMER_2     true
  #warning Using Timer1
#endif

#define LED             13  
#define LED2            10 //PB2


//LIBRARIES
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <TimerInterrupt.h>
#include <Arduino.h>

//Variables
const unsigned long interval = 10; //In ms
const unsigned long duration = 0;   //How many cycles do you want, 0 for infinite
bool  Toggle=false;


//Functions
void TimerHandler();
void FASTdigitalWrite(uint8_t PIN, uint8_t VALUE);
uint8_t FASTdigitalRead(uint8_t PIN); 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED,OUTPUT);
  pinMode(LED2, OUTPUT);

  //TIMER CONFIGURATION  
  ITimer2.init();
  // interval (in ms) and duration (in milliseconds). Duration = 0 or not specified => run indefinitely
  if (ITimer2.setInterval(interval, TimerHandler, duration))
  {
    digitalWrite(LED2,HIGH);
  } else
  {
    digitalWrite(LED2,LOW);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Toggle)
  {
    digitalWrite(LED,HIGH);
  } else 
  {
    digitalWrite(LED,LOW);
  }
}

//INTERRUPT ROUTINE
void TimerHandler()
{
  // Doing something here inside ISR
  Toggle = !Toggle; //Invert toggle

}





