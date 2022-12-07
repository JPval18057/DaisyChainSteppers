//Definitions
#define LED LED_BUILTIN
//Variables
bool ledstate=false;
long current,previous;
uint8_t interval=500;
int count;

//Functions
void coms_config();


void setup() {
  // put your setup code here, to run once:
  coms_config();
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  current = millis();
    if ((current-previous)>=interval){
      count++;
    }
}

void coms_config(){
  Serial.begin(115200,SERIAL_8N1);
  while(!Serial){
    //Wait for the module to get ready
  }

}
  
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == 65) {
      ledstate = true;
      Serial.write('A');
      //Serial.flush(); //wait until the data is sent
    }
  }
}
