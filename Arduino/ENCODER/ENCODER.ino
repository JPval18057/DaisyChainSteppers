/*
 * Author: Juan Pablo Valenzuela
 * Description:
 * Code that debounces and gets data from the encoder
 * Date: 9/12/2022
 */
//****************************************************************************************
 //Libraries


//****************************************************************************************
//Definitions

// Rotary Encoder Module connections
const int PinSW=9;   // Rotary Encoder Switch
const int PinDT=8;    // DATA signal
const int PinCLK=7;    // CLOCK signal

#define hall_switch 2
#define LED 10
//PIN 8 is the pin where the sensor reading is taken from

//****************************************************************************************


//****************************************************************************************
//Variables

int Switch_state;
bool button=0;
String mssg;

//Debouncing button
bool buttonState = LOW; 

// Variables to debounce Rotary Encoder
long TimeOfLastDebounce = 0;
int DelayofDebounce = 0.010; //Debounce delay of 10ms the average of all systems

// Store previous Pins state
int PreviousCLK;   
int PreviousDATA;

int displaycounter=0; // Store current counter value
int angle = 0; //Angle of rotation
int desired_angle = 0;

//****************************************************************************************

//****************************************************************************************
//Function prototypes
void check_rotary(void);
void Encoder_config(void);

boolean debounceButton(boolean state);
void checkswitch(void);

void Hall_config(void);
void reset_pos(void);
//****************************************************************************************

void setup() {
  // put your setup code here, to run once:
  Encoder_config();
}

void loop() {
  // put your main code here, to run repeatedly:  
  // If enough time has passed check the rotary encoder
  if ((millis() - TimeOfLastDebounce) > DelayofDebounce) {
    
    check_rotary();  // Rotary Encoder check routine below
    
    PreviousCLK=digitalRead(PinCLK);
    PreviousDATA=digitalRead(PinDT);
    
    TimeOfLastDebounce=millis();  // Set variable to current millis() timer
  }
  checkswitch(); //check if button was pressed
  if (digitalRead(hall_switch)==HIGH){
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
}

void Encoder_config(){
  // Put current pins state in variables
  PreviousCLK=digitalRead(PinCLK);
  PreviousDATA=digitalRead(PinDT);

  // Set the Switch pin to use Arduino PULLUP resistors
  pinMode(PinSW, INPUT); //Not necesary since the encoder has a pullup resistor
  pinMode(PinDT, INPUT);
  pinMode(PinCLK, INPUT);
  pinMode(LED, OUTPUT); //for visual aid
  pinMode(hall_switch,INPUT);
  Serial.begin(115200);
}

// Check if Rotary Encoder was moved
void check_rotary() {

 if ((PreviousCLK == 0) && (PreviousDATA == 1)) {
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 0)) {
      displaycounter++;
      angle++;
     Serial.println(displaycounter);
     //Serial.println(int(angle*(360.0/40.0)));
    }
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 1)) {
      displaycounter--;
      angle--;
      Serial.println(displaycounter);
      //Serial.println(int(angle*(360.0/40.0)));
    }
  }

if ((PreviousCLK == 1) && (PreviousDATA == 0)) {
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 1)) {
      displaycounter++;
      angle++;
      Serial.println(displaycounter);
      //Serial.println(int(angle*(360.0/40.0)));
    }
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 0)) {
      displaycounter--;
      angle--;
     Serial.println(displaycounter);
     //Serial.println(int(angle*(360.0/40.0)));
    }
  }

if ((PreviousCLK == 1) && (PreviousDATA == 1)) {
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 1)) {
      displaycounter++;
      angle++;
      Serial.println(displaycounter);
      //Serial.println(int(angle*(360.0/40.0)));
    }
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 0)) {
      displaycounter--;
      angle--;
     Serial.println(displaycounter);
     //Serial.println(int(angle*(360.0/40.0)));
    }
  }  

if ((PreviousCLK == 0) && (PreviousDATA == 0)) {
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 0)) {
      displaycounter++;
      angle++;
      Serial.println(displaycounter);
      //Serial.println(int(angle*(360.0/40.0)));
    }
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 1)) {
      displaycounter--;
      angle--;
      Serial.println(displaycounter);  
      //Serial.println(int(angle*(360.0/40.0)));  
    }
  }

  }


boolean debounceButton(boolean state){
  bool stateNow = digitalRead(PinSW);
  if(state!=stateNow)
  {
    delay(10);
    stateNow = digitalRead(PinSW);
  }
  return stateNow;
  
}

void checkswitch(){
  if(debounceButton(buttonState) == HIGH && buttonState == LOW)
  {
    Serial.println("Pressed");
    buttonState = HIGH;
  }
  else if(debounceButton(buttonState) == LOW && buttonState == HIGH)
  {
       buttonState = LOW;
  }
}
