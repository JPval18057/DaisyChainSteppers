/*
 * Author: Juan Pablo Valenzuela
 * Description: Code for the motors that
 * Calibrates the motor (homing the stepper)
 * Sets the position
 * Contains the PID controller
 * Communicates with master and other motors
 * 
 */

//Libraries
//Stepper motor & driver
#include "AccelStepper.h" 
//I2C-communication
#include <Wire.h>

//Preprocessor instructions
// AccelStepper Setup
#define dirPin 11
#define stepPin 12
#define motorInterfaceType 1
#define home_switch 9

//Variables

//Functions
void daisy_config(void); //Daisy protocol configuration
void calibrate(int home_switch);

//Main code


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
