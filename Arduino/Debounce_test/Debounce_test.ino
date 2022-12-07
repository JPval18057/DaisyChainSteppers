/*
 * Author: Juan Pablo Valenzuela
 * Description: SCHMIDT TRIGGER DEBOUNCE TEST 
 * 5.1k and 1k resistors with a 10uF capacitor
 */
#define LED LED_BUILTIN
#define SWITCH 2
//Interrupt pins 2 and 3
bool ledstate,print_mssg;
String mssg;
uint8_t count;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  pinMode(SWITCH, INPUT);
  attachInterrupt(digitalPinToInterrupt(SWITCH), button, RISING);
  Serial.begin(115200);
  while(!Serial){
    //wait for serial module to be ready
  }
  Serial.print("\nArduino ready!\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (print_mssg) {    
      mssg = "\nButton detected, count: " + String(count)+"\n";
      Serial.print(mssg);
      print_mssg=false;
  }
}

void button(){
  count++;
  print_mssg=true;
}
