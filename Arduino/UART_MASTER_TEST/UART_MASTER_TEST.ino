//Library
#include <HardwareSerial.h>
HardwareSerial SerialPort(2);  //if using UART2

//Definitions
#define LED 2
#define BaudRate 115200
#define SerialMode SERIAL_8N1
#define RX_pin 16 
#define TX_pin 17



//Variables
long current,previous;
const long interval = 1000;
String serialbuffer = "";
int swap;

//Functions
void coms_config();

void setup() {
  // put your setup code here, to run once:
  coms_config();
}

void loop() {
  // put your main code here, to run repeatedly:
  current = millis();
  if ((current - previous)>interval){
    SerialPort.print(65);
    previous = current;
  }
}

void coms_config(){
  //Objects
  SerialPort.begin(BaudRate, SerialMode, RX_pin, TX_pin);
  
}

void serialEvent() {
  while (SerialPort.available()) {
    // get the new byte:
    char inChar = SerialPort.read();
    
  }
}
