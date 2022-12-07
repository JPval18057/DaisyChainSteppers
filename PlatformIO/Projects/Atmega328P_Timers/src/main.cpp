/*  Timers with interruption test
*   Author: Juan Pablo Valenzuela
*   Description: This is the timer interruption example code
*/

//Libraries
#include <Arduino.h>

//Preprocessor directives
#define LED 10 //13 is the built in LED

#define clock_freq  16000000 //Clock frequency
#define baudrate  115200 //UART BAUDRATE

//Objects

//Functions
void Timer0Config();
void Timer1Config();
void Timer2Config();
void InterruptConfig();

//##############################################################################################
//                                  Variables
//##############################################################################################
//Time measurements
//TIMER 0 _____ VALUE
const byte TMR0 = 0xEB; //it has only one byte (8bit timer)
//Interval of 10 us

//TIMER 1 TCNT1 VALUES
const byte TMR1L = 0xFF; //low byte
const byte TMR1H = 0xE6; //high byte
//Interval of 100 us

//TIMER 2 _____ VALUE
const byte TMR2 = 0xEB; //it is a 8bit timer
//Interval of 10 us
//You can modify them to adjust the time to your needs
//See the Excel sheet with the time calculations

//Debug
uint8_t ledstate = LOW;
long T2COUNT = 0;
uint8_t T2LOOP = 0; //Program timer control variables
uint8_t T0LOOP = 0;
uint8_t T1LOOP = 0;

//Use TIMER 2 FOR SAMPLING TIME AND TIMER 1 FOR DEBOUNCE INTERVAL

//##############################################################################################

void setup() {
  // put your setup code here, to run once:
  Timer0Config();
  // Timer1Config();
  // Timer2Config();
  InterruptConfig();
  pinMode(LED, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:  
  if ((T0LOOP==HIGH)){
    ledstate = !ledstate;
    T2LOOP = LOW;
    digitalWrite(LED, ledstate); //tarda 2 us en ejecutarse, es lenta

  }

}

//##############################################################################################
//                                  TIMER FUNCTIONS
//##############################################################################################
//TIMER 0 CONFIGURATION
void Timer0Config()
{
/*
   * Timer 0 is an 8 bit timer
   * counts from 0 to 255
   * can be used on match or overflow mode
   * cannot be triggered by external pins
   */
   //Initialize timer 0 on default values
   TCCR0A = 0;
   TCCR0B = 0;
   TCCR0B |= 0b00000010; //Prescaler of 8 p.87
   TIMSK0 |= 0b00000001; //Last bit enables overflow interrupt p.88
   //Loading starting time
   TCNT0 = TMR0;
}
//TIMER 1 CONFIGURATION
void Timer1Config()
{
/*
   * Timer 1 is a 16 bit timer
   * counts from 0 to 65,535
   * can be used on match or overflow interrupt
   */
  TCCR1A = 0; //set everything to 0 to default the timer
  TCCR1B = 0; //The last 3 bits specify the clock source, p112 datsheet
  TCCR1B |= 0b00000011; //Sets the prescaler to 64 and activates clock source (page 110)
  TIMSK1 |= 0b00000001; //Activate overflow interrupt last bit
  //LOADING START TIME (It's the same every time because the interval is the same)
  TCNT1L = TMR1L;
  TCNT1H = TMR1H;
}
//TIMER 2 CONFIGURATION
void Timer2Config()
{
/*
   * Timer 2 is an 8 bit timer
   * counts from 0 to 255
   * can be used on match or overflow mode
   * cannot be triggered by external pins
   */
   //Initialize timer 0 on default values
   TCCR2A = 0;
   TCCR2B = 0;
   TCCR2B |= 0b00000010; //Prescaler of 8 p.87
   TIMSK2 |= 0b00000001; //Last bit enables overflow interrupt p.88
   //Loading starting time
   TCNT2 = TMR2;
   
}
//INTERRUPT ENABLE
void InterruptConfig()
{
  interrupts();
}

//TIMER 0 AND TIMER 2 WORK EXACTLY THE SAME, YOU CAN USE THE SAME CALCULATOR FOR THEM

//Interrupt Service Routines
//TIMER 0
ISR(TIMER0_OVF_vect){
  //INTERRUPT FUNCTION ON OVERFLOW FOR TIMER 0 (8 bit timer)
  TCNT0 = TMR0;
  //THE CODE THAT EXECUTES ON INTERRUPT GOES HERE
  T0LOOP = HIGH;

}

// //TIMER 1
// ISR(TIMER1_OVF_vect){
//   //INTERRUPT FUNCTION ON OVERFLOW FOR TIMER 1 (16 bit mode)
//   TCNT1L = TMR1L;
//   TCNT1H = TMR1H;
//   //THE CODE THAT EXECUTES ON INTERRUPT GOES HERE
//   ledstate = ~ledstate;

// }

// //TIMER 2
// ISR(TIMER2_OVF_vect){
//   //INTERRUPT FUNCTION ON OVERFLOW FOR TIMER 0 (8 bit timer)
//   TCNT2 = TMR2;
//   //THE CODE THAT EXECUTES ON INTERRUPT GOES HERE
//   T2COUNT++;
//   T2LOOP = HIGH;
//   if (T2COUNT==100000){
//     T2COUNT=0;
//     //OVERFLOW THE COUNTER TO PREVENT IT FROM RUNNING OUT OF CONTROL
//   }

// }