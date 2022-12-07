/* Arduino New Rotary Encoder Debounce
 
Created by Yvan /  https://www.brainy-bits.com/post/best-code-to-use-with-a-ky-040-rotary-encoder-let-s-find-out

This code is in the public domain...

You can: copy it, use it, modify it, share it or just plain ignore it!
Thx!

*/

// Rotary Encoder Module connections
const int PinSW=4;   // Rotary Encoder Switch
const int PinDT=3;    // DATA signal
const int PinCLK=2;    // CLOCK signal

// Variables to debounce Rotary Encoder
long TimeOfLastDebounce = 0;
int DelayofDebounce = 0.010; //Debounce delay of 10ms the average of all systems

// Store previous Pins state
int PreviousCLK;   
int PreviousDATA;

int displaycounter=0; // Store current counter value
int angle = 0; //Angle of rotation
int desired_angle = 0;

void setup() {
  // Put current pins state in variables
  PreviousCLK=digitalRead(PinCLK);
  PreviousDATA=digitalRead(PinDT);

  // Set the Switch pin to use Arduino PULLUP resistors
  pinMode(PinSW, INPUT_PULLUP); //Not necesary since the encoder has a pullup resistor
  pinMode(PinDT, INPUT);
  pinMode(PinCLK, INPUT);
  Serial.begin(115200);

}
void loop() {
  // If enough time has passed check the rotary encoder
  if ((millis() - TimeOfLastDebounce) > DelayofDebounce) {
    
    check_rotary();  // Rotary Encoder check routine below
    
    PreviousCLK=digitalRead(PinCLK);
    PreviousDATA=digitalRead(PinDT);
    
    TimeOfLastDebounce=millis();  // Set variable to current millis() timer
  }
  
  
}


// Check if Rotary Encoder was moved
void check_rotary() {

 if ((PreviousCLK == 0) && (PreviousDATA == 1)) {
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 0)) {
      displaycounter++;
     Serial.println(displaycounter);
    }
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 1)) {
      displaycounter--;
      Serial.println(displaycounter);
    }
  }

if ((PreviousCLK == 1) && (PreviousDATA == 0)) {
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 1)) {
      displaycounter++;
      Serial.println(displaycounter);
    }
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 0)) {
      displaycounter--;
     Serial.println(displaycounter);
    }
  }

if ((PreviousCLK == 1) && (PreviousDATA == 1)) {
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 1)) {
      displaycounter++;
      Serial.println(displaycounter);
    }
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 0)) {
      displaycounter--;
     Serial.println(displaycounter);
    }
  }  

if ((PreviousCLK == 0) && (PreviousDATA == 0)) {
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 0)) {
      displaycounter++;
      Serial.println(displaycounter);
    }
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 1)) {
      displaycounter--;
      Serial.println(displaycounter);    
    }
  }

              // Check if Rotary Encoder switch was pressed
  if (digitalRead(PinSW) == LOW) {
    displaycounter=0;  // Reset counter to zero
   Serial.println("Pressed switch");
  }
 }
