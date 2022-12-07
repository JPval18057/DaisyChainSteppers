#include <SPI.h>

//Definitions
//#define SS    53
//#define CLK   52
//#define MOSI  51
//#define MISO  50

#define BUFFERSIZE 32
//Variables
bool received = false;
uint8_t device_number;
char data_in[BUFFERSIZE];
long current,previous;
const long interval=1000;
  
//Functions
void calibrate();
void spi_config();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial){
    //Wait until the module is up and running
  }
  Serial.println("\nArduino ready!\n");
//  Serial.print("\nMosi");Serial.print(MOSI);
//  Serial.print("\nMiso");Serial.print(MISO);
//  Serial.print("\nSS");Serial.print(SS);
//  Serial.print("\nSCK");Serial.print(SCK);
  //calibrate();
  spi_config();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  current = millis();
  if ((current-previous)>= interval){
    previous = current;
    digitalWrite(SS,LOW);
    data_in[0] = SPI.transfer(65);
    digitalWrite(SS,HIGH);
    if (data_in[0]==65){
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
    Serial.print("\n");Serial.print(char(data_in[0]),DEC);Serial.print("\n");
  }

}

void calibrate(){
  pinMode(SS,OUTPUT); //CHIP SELECT AS OUTPUT BECAUSE THIS IS THE MASTER
  pinMode(MISO, INPUT);
  digitalWrite(SS,HIGH); //Disable communications for now
  SPI.begin();  
  //SPI.setBitOrder(LSBFIRST);
  //SPCR = 0x70;                       //Turn on SPI in MASTER Mode MEGA datasheet page 197 fosc/4
  //and interrupt disabled
  //Start communication
  digitalWrite(SS,LOW); //Enable communications
  bool calibration=1;

  while(calibration){
    
    data_in[device_number] = SPI.transfer(65); //READ BYTE FROM SPI DATA REGISTER
    if(data_in[device_number]==65 | device_number==BUFFERSIZE){
      calibration=0;//if the data comes back then finish calibration
    }
    device_number++;
  }
  digitalWrite(SS,HIGH);
  Serial.println("Done");
  String mssg;
  for (int i;i<BUFFERSIZE;i++){
    mssg = "Data in buffer: ";
    Serial.println(mssg); Serial.print(data_in[i]);
  }
}

void spi_config(){
  pinMode(SS,OUTPUT); //CHIP SELECT AS OUTPUT BECAUSE THIS IS THE MASTER
  pinMode(MISO, INPUT);
  digitalWrite(SS,HIGH); //Disable communications for now
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16); //1MHz frecuency for the SPI
  delay(10); //wait a little before sending data
}
//ISR (SPI_STC_vect){                        //Inerrrput routine function 
//  data_in[0]=SPDR;                   // Store value received in data_received
//  received = true;                        //Sets received as True 
//}
