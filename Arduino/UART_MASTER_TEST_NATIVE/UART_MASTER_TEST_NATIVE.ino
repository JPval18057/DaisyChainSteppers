/*
 * This is the only uart code that works, if it works it stays
 */
//Definitions
#define LED 2
#define LED2 23
#define BaudRate 115200
#define SerialMode SERIAL_8N1
#define RX_pin 16 
#define TX_pin 17



//Variables
long current,previous;
const long interval = 1000;
String serialbuffer = "";
int swap;
bool sendData=0;
bool countDevices=0;

//Functions
void coms_config();

void setup() {
  // put your setup code here, to run once:
  coms_config();
  pinMode(LED, OUTPUT);
  pinMode(LED2,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  current = millis();
  if ((current - previous)>interval){
    swap++;
    sendData=1;
    previous = current;
  }
  if (swap==1 & sendData==1){
    Serial2.write(65);
    digitalWrite(LED, HIGH);
    sendData=0;
  }
  if (swap==2 & sendData==1){
    Serial2.write(70);
    digitalWrite(LED, LOW);
    sendData=0;
  }
  if (swap==3){
    swap = 0;
  }
}

void coms_config(){
  //Objects
  Serial2.begin(BaudRate,SerialMode);
  Serial.begin(BaudRate,SerialMode); //use the same configuration for uart0
  
}

//There is serialEvent, serialEvent1, serialEventt2 for boards with multiple uart modules
void serialEvent2() {
  while (Serial2.available()) {
    // get the new byte:
    char inChar = Serial2.read();
    if (inChar==65){
      digitalWrite(LED2, HIGH);
    } else {
      digitalWrite(LED2, LOW);
    }
    
  }
}

void serialEvent(){
  while (Serial.available()){
    char inChar = Serial.read();
    if (inChar=='C'){
      countDevices=true;
    }
  }
}
