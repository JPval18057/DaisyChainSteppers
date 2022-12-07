/*
 * Author:
 * Description:
 * This is for the ESP32
 * A very good tutorial on how to use SPI on the ESP32
 * https://microcontrollerslab.com/esp32-spi-communication-tutorial-arduino/
 * https://randomnerdtutorials.com/esp32-spi-communication-arduino/#spi-multiple-bus-hspi-vspi
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
//SPI CONFIGURATION
static const int spiClk = 1000000; // 1 MHz
//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;

//Variables
byte id=0;
byte calibration_data;

//Interval
long current_value,previous_value;
int interval = 1000;

//Function definitions
void calibrate();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.print("\nDevice started\n");
  
  calibrate(); //calibration of the protocol
  
}

void loop() {
  current_value = millis();
  if ((current_value - previous_value)>=interval){
    //Stuff to do every interval
        
    previous_value = current_value;
  }

}

//This function assigns each device it's number
void calibrate(){
  /*
   * Visual indication of calibration
   */
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  /*
   * This is the configuration of the SPI communication
   */
  byte device_number, received_data;  
  vspi = new SPIClass(VSPI);
  vspi->begin(CLK, MISO, MOSI, SS); //make sure the pins we are using are the ones doing stuff
  pinMode(SS, OUTPUT); //chip select pin as output to control data transfer
  digitalWrite(SS,HIGH); //Leave it on High to prevent any communication
  /*
   * Start the identification protocol
   */
  vspi->beginTransaction(SPISettings(spiClk, LSBFIRST, SPI_MODE0)); //I think LSB first is better because it doesn't flip the number on the receiving end
  digitalWrite(SS, LOW); //SS on LOW to allow communication between modules
  bool calibration = 1;
  while (calibration){
    received_data = vspi->transfer('A'); //Begin transfering data
    //To read the data in the buffer refer to this document
    //https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#spi
    //page 119 where it says slave status register
    device_number++;
    if(received_data=='A' | device_number==30){
      calibration=0; //if the data reaches full circle then stop the process
    }
    delay(10); //wait 10ms just in case
  }
  vspi->endTransaction(); //End data transfer
  digitalWrite(led,LOW);
  /*
   * Visual indication of calibration
   */
  Serial.println(received_data);
  String mssg = "Devices detected: " + String(device_number);
  Serial.println(mssg);
  
  
  //The number of connected devices is device_number and could be returned as the function value
}
