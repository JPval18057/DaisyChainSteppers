// Include the AccelStepper library:
/*
 * Reference:
 * https://www.makerguides.com/drv8825-stepper-motor-driver-arduino-tutorial/
 */
#include <AccelStepper.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 11
#define stepPin 12
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // Set the maximum speed in steps per second:
  stepper.setMaxSpeed(4000); //steps/sec
  stepper.setAcceleration(100);
}

void loop() {
  // Set the speed in steps per second:
  stepper.setSpeed(100);
  // Step the motor with a constant speed as set by setSpeed():
  stepper.runSpeed();
}

/*
 * Investigate all the functions in this library
 */
