/*
 * Author:
 * Description:
 * This is for the ESP32
 */
//Libraries
#include <SPI.h>

/*
 * Default VSPI pins
 * MOSI Pin: 23
 * MISO Pin: 19
 * SCK Pin: 18
 * SS Pin: 5
 * Default HSPI pins
 * MOSI Pin: 13
 * MISO Pin: 12
 * SCK Pin: 14
 * SS Pin: 15
 */
 
//Pin definitions
//MASTER OUT SLAVE IN
#define MOSI 23
//MASTER IN SLAVE OUT
#define MISO 19
//CHIP SELECT
#define SS 5
//CLOCK FOR DATA TRANSFER
#define CLK 18
//Builtin LED for ESP32
#define led 2

//Variables
byte id=0;
byte calibration_data;

//Interval
long current_value,previous_value;
int interval = 1000;

//Function definitions
byte calibrate();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.print("\nDevice started, wait 2s\n");
  delay(2000);  
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  id = calibrate();  
  String mssg = "Devices detected: " + String(id);
  Serial.println(mssg);
  digitalWrite(led,LOW);
}

void loop() {
  current_value = millis();
  if ((current_value - previous_value)>=interval){
    //Stuff to do every interval
        
    previous_value = current_value;
  }

}

//This function assigns each device it's number
byte calibrate(){
  byte device_number;
  char received_data;
  pinMode(SS, OUTPUT); //chip select pin as output to control data transfer
  SPI.begin(); //CLK, MISO, MOSI, SS
  SPI.setClockDivider(SPI_CLOCK_DIV8);    
  digitalWrite(SS,HIGH); //Make SS HIGH to prevent any comunication for now
  bool calibration=1;
  digitalWrite(SS,LOW); //START COMMUNICATION FOR MASTER
  while (calibration) {    
    received_data = SPI.transfer('A');
    device_number++;  
    Serial.println(received_data);
    if (received_data=='A' | device_number==30){
      calibration=0;
    }
    delay(10); //wait 10ms before trying again
  }
  digitalWrite(SS,HIGH); //Make SS HIGH to prevent any comunication for now
  //The number of connected devices is device_number and could be returned as the function value
  return device_number;
}
