// Include the AccelStepper library:
/*
 * Reference:
 * https://www.makerguides.com/drv8825-stepper-motor-driver-arduino-tutorial/
 * Functions
 * https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#a344f58fef8cc34ac5aa75ba4b665d21c
 */
#include <AccelStepper.h>


// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 11
#define stepPin 12
#define motorInterfaceType 1
#define hall_switch 2  //10 is the default
#define LED 10

long previous,current,time_count;
long interval = 500;
int stepper_speed = 50;

// Create a new instance of the AccelStepper class:
AccelStepper stepperX(motorInterfaceType, stepPin, dirPin);
/*
 * The unipolar stepper can move up to 150 steps/minute before failing
 * The bipolar can move much faster
 * The unipolar stepper has a specific connection
 * B2 Blue
 * B1 Red
 * A1 Green
 * A2 Black
 * The maximum speed possible is 600 steps/s it seems we need a bipolar driver to run it accurately
 */

//Functions
void calibrate();

void setup() {
  // Set the maximum speed in steps per second:
  stepperX.setMaxSpeed(1000); //steps/sec the max speed at 16MHz is 4000 steps/sec
  stepperX.setAcceleration(100); //steps/sec^2
  pinMode(hall_switch, INPUT);
  pinMode(LED, OUTPUT);
  calibrate();
}

void loop() {
  current = millis();
  if ((current-previous)>=interval){
    time_count++;    
    stepper_speed = stepper_speed + 10;
    previous = current;
  }  
  //Just to know if the program is running
  if (time_count%10==0){
    time_count = 0;
    stepperX.moveTo(-100);
  } else {
      stepperX.moveTo(100);
  }
  stepperX.run(); //must be called repetitively to make the motor move


}

//This function worked because I changed the driver
//Never use the motors without the lab voltage sources
void calibrate(){
  //Rotate Clock Wise while switch reading is 0
  while (digitalRead(hall_switch)==LOW) {
    stepperX.move(1);
    stepperX.run();
  }
  //Rotate Counter Clock Wise while switch readidng is 1
  while (digitalRead(hall_switch)==HIGH) {
    stepperX.move(-1);
    stepperX.run();
  }
  //Set zero position
  stepperX.setCurrentPosition(0);
  stepperX.setMaxSpeed(1000); //steps/sec the max speed at 16MHz is 4000 steps/sec
  stepperX.setAcceleration(100); //steps/sec^2
}


/*
 * Investigate all the functions in this library
 * https://www.pjrc.com/teensy/td_libs_AccelStepper.html
 */
