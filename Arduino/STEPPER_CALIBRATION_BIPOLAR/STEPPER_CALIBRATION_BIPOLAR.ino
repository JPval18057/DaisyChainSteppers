/*  Motor Homing code using AccelStepper and the Serial Monitor
 
Created by Yvan / https://Brainy-Bits.com
This code is in the public domain...
You can: copy it, use it, modify it, share it or just plain ignore it!
Thx!

*/

#include <AccelStepper.h>
// Library created by Mike McCauley at http://www.airspayce.com/mikem/arduino/AccelStepper/

// AccelStepper Setup
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 11
#define stepPin 12
#define motorInterfaceType 1

AccelStepper stepperX = AccelStepper(dirPin, stepPin, motorInterfaceType);   // 1 = Easy Driver interface
                                  // NANO Pin 2 connected to STEP pin of Easy Driver
                                  // NANO Pin 3 connected to DIR pin of Easy Driver

// Define the Pins used
#define home_switch 10 // Pin 9 connected to Home Switch (MicroSwitch)

// Stepper Travel Variables
long TravelX;  // Used to store the X value entered in the Serial Monitor
int move_finished=1;  // Used to check if move is completed
long initial_homing=1;  // Used to Home Stepper at startup
bool perform_calibration = 1;
void calibrate(void);

void setup() {
   //Serial.begin(9600);  // Start the Serial monitor with speed of 9600 Bauds
  
  pinMode(home_switch, INPUT);
  delay(5);  // Wait for EasyDriver wake up

   //  Set Max Speed and Acceleration of each Steppers at startup for homing
  stepperX.setMaxSpeed(200.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepperX.setAcceleration(200.0);  // Set Acceleration of Stepper
  stepperX.setCurrentPosition(0);  // Set the current position as zero for now
   

}

void loop() {
//  if (perform_calibration){
//    calibrate();
//    perform_calibration = 0;
//  }
  stepperX.setSpeed(100);
  stepperX.run();
}

void calibrate(void){
     delay(5);  // Wait for EasyDriver wake up

   //  Set Max Speed and Acceleration of each Steppers at startup for homing
  stepperX.setMaxSpeed(200.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepperX.setAcceleration(200.0);  // Set Acceleration of Stepper
  stepperX.setCurrentPosition(0);  // Set the current position as zero for now

// Start Homing procedure of Stepper Motor at startup
bool calibration = 1;
  while (calibration) {  // Make the Stepper move CCW until the switch is activated   
    stepperX.setSpeed(100); //move slowly until switch is activated
    stepperX.run();  // Start moving the stepper
    if (digitalRead(home_switch)==1){
      calibration = 0;
    }
    delay(5);
}

  stepperX.setCurrentPosition(0);  // Set the current position as zero for now
  stepperX.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepperX.setAcceleration(100.0);  // Set Acceleration of Stepper
calibration = 1;
  while (calibration) { // Make the Stepper move CW until the switch is deactivated
    stepperX.setSpeed(-100);  
    stepperX.run();
    if (digitalRead(home_switch)==0){
      calibration = 0;
    }
    delay(5);
  }

  //HOMING COMPLETED
  stepperX.setCurrentPosition(0);
  stepperX.setMaxSpeed(1000.0);      // Set Max Speed of Stepper (Faster for regular movements)
  stepperX.setAcceleration(1000.0);  // Set Acceleration of Stepper

}
