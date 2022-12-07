/*
 * Author:
 * Description:
 * This is the code for each slave microcontroller
 */
//Libraries
#include <SPI.h>

//Pin definitions
//MASTER OUT SLAVE IN
#define MOSI 23
//MASTER IN SLAVE OUT
#define MISO 19
//CHIP SELECT
#define SS 5
//CLOCK FOR DATA TRANSFER
#define CLK 18

//Variables
volatile boolean received;
byte data_received;
bool ledstate=false;

//Functions
void calibrate();

void setup() {
  // put your setup code here, to run once:
  calibrate();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (received){
    received=false;    
    ledstate=false;
  }
  digitalWrite(LED_BUILTIN,ledstate);
  
}

void calibrate(){
  //pinMode(SS, INPUT);
  SPI.begin();
  pinMode(MISO,OUTPUT);                   //Sets MISO as OUTPUT (Have to Send data to Master IN 
  pinMode(SS, INPUT);
  SPCR = 0xE0;                       //Turn on SPI in Slave Mode datasheet page 141
  received = false;
  SPI.attachInterrupt();                  //Interuupt ON is set for SPI commnucation
}

ISR (SPI_STC_vect){                        //Inerrrput routine function 
  data_received = SPDR;                   // Store value received in data_received
  SPDR = 'A';
  received = true;                        //Sets received as True 
  ledstate = true;
}
