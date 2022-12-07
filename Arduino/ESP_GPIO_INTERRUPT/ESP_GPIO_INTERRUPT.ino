/*
 * Author: Juan Pablo Valenzuela
 * Description: IO TEST ESP32
 * Remember this board runs at 3.3V
 */
 //bits definition
#define M0 22
#define M1 1
#define M2 3
#define M3 21
 
#define M4 36
#define M5 39
#define M6 34
#define M7 35

#define bytesize 8

//Fake register setup
byte M_reg[bytesize] = {M0,M1,M2,M3,M4,M5,M6,M7};
byte M_values=0;

//time count
long current,previous,count;
const int interval = 1000; //count every 1000 ms
bool shift_done=0;

//functions
void setport(byte value);

void setup() {
  // put your setup code here, to run once:
  for (int i;i<bytesize;i++) {
    pinMode(M_reg[i],OUTPUT);
  }
  setport(LOW);
  Serial.begin(115200);
  while(!Serial);
}

void loop() {
  // put your main code here, to run repeatedly:
  current = millis();
  if ((current-previous)>interval){
    count++;
    shift_done = 1;
    previous = current;
  }
  if (shift_done){
    shift_done = 0;
    setport(LOW);
  }
}



void setport(byte value){
  for (int i;i<bytesize;i++) {
    digitalWrite(M_reg[i],value);
  }
}
