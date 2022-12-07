//***********************************************************************************************
/*
Title: Daisy-Chain Slave Code
Author: Juan Pablo Valenzuela
Date: 10/9/2022
Description:
This code controls the Slave microcontroller, an Atmega328.
This code manages the communication protocol, PID controllers and sensors debounce.
*/
//***********************************************************************************************

//***********************************************************************************************
//Macros
//***********************************************************************************************
//Built in LED is 13 on atmega328
#define LED A5          
#define LED2 A4
//EUSART Universal Baudrate for this project
#define CommsBaudrate 115200 //1,000,000
//Is 64 bytes by default but the JSON Assistant recommends 24 bytes for serializing and 64 for deserializing
#define BUFFER_SIZE 128       
#define MASTER_ID 0 //This is the default master ID

//MOTOR CONTROL
#define dirPin 11
#define stepPin 12
#define motorInterfaceType 1
#define hall_switch 9  //10 is the default

//Encoder Debbug
#define encoderDebug 1


//Interrupt Timer Marcos
// These define's must be placed at the beginning before #include "TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#define USE_TIMER_1     false

#if ( defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)  || \
        defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) ||    defined(ARDUINO_AVR_ETHERNET) || \
        defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT)   || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO)      || \
        defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) || \
        defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
        defined(ARDUINO_AVR_PROTRINKET3FTDI) )
  #define USE_TIMER_2     true
  #warning Using Timer1
#endif

//***********************************************************************************************
//Libraries
//***********************************************************************************************
//Mathematical operations for the PID controller
#include <math.h>
#include <Arduino.h>      //It's obvious why this is here (Arduino board -> Arduino lib)
//Comms
#include <ArduinoJson.h>  //Necessary for the Daisy Chain Library to work
#include <Daisychain.h> //My own daisychain library
//Stepper control
#include <AccelStepper.h> //Stepper control (uses micros() -> timer0) check if it uses servo.h
#include <EEPROM.h> //In case we need to save something to memory
#include <TimerInterrupt.h> //Be very careful using this, check the example
//Watch out for the definitions prior to the #include

//***********************************************************************************************
//Objects
//***********************************************************************************************
//JSON DOCUMENT
//Document to store incoming information
StaticJsonDocument<BUFFER_SIZE> Inputdoc;
//Document to store outgoing information
StaticJsonDocument<BUFFER_SIZE> Outputdoc;

//Stepper Object
AccelStepper stepperX(motorInterfaceType, stepPin, dirPin);

//***********************************************************************************************
//Variables
//***********************************************************************************************
//DaisyChain Protocol
//Default values
//This is the variable that stores the command to send
command_t command = CMD_NULL;
//This is the device id received
uint8_t id = 0;
//This is the payload mainly for PID constants
float payload = 1.0000;

//EEPROM VARIABLES
//Device EEPROM ID
uint8_t EEPROM_ID; 
uint8_t EEPROM_CONTROL_MODE = POSITION;
//GEAR_RATIO
double EEPROM_GEAR_RATIO = 1.0; //UPDATE THIS VALUE IN SETUP (MAKE IT DOUBLE FOR ADDED FLEXIBILITY)

//EEPROM ADDRESSES
uint8_t FLOAT_SIZE = 4; //This is the size in bytes of a float data type
uint8_t INT_SIZE = 2; //This is the size in bytes of an integrer data type
uint8_t DOUBLE_SIZE = 4; //In this board doubles are the same size as floats
uint8_t EEPROM_ID_ADDRESS = 0;
//Position
uint8_t EEPROM_KP_ADDRESS = EEPROM_ID_ADDRESS+1;
uint8_t EEPROM_KI_ADDRESS = EEPROM_KP_ADDRESS+DOUBLE_SIZE;
uint8_t EEPROM_KD_ADDRESS = EEPROM_KI_ADDRESS+DOUBLE_SIZE;
//Speed
uint8_t SPEED_EEPROM_KP_ADDRESS = EEPROM_KD_ADDRESS+DOUBLE_SIZE;
uint8_t SPEED_EEPROM_KI_ADDRESS = SPEED_EEPROM_KP_ADDRESS+DOUBLE_SIZE;
uint8_t SPEED_EEPROM_KD_ADDRESS = SPEED_EEPROM_KI_ADDRESS+DOUBLE_SIZE;
//Control Mode
uint8_t EEPROM_MODE_ADDRESS = SPEED_EEPROM_KD_ADDRESS+DOUBLE_SIZE;
uint8_t EEPROM_GEAR_RATIO_ADDRESS = EEPROM_MODE_ADDRESS+DOUBLE_SIZE;



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

//Speed measurement variable
double RPMCount;
int displaycounter=0; // Store current counter value
float angle = 0; //Angle of rotation = 360/(steps per rotation)*displaycounter
float desired_angle = 18; //Desired angle in degrees
const double steps_per_revolution = 200; //In a 1:1 encoder gear setup this is 40 steps/rev but since we have a 5:1 ratio this is now 200


//PID Controller
//Position
double kp, kd, ki; //In the function PID_CONFIG() they are given initial values
double err,err_prev,err_ki,err_kd; //All these initialize in 0
double uk; //Work variable, just saves the PID calculations as a working register
int MotorSpeed = 100; //Value to write in stepperX.setSpeed();
double Motorposition = 90; //Desired position (displaycounter steps)
// deg = 360 deg/200step * (displaycounter steps)
double PositionControlResolution = (360/(steps_per_revolution*EEPROM_GEAR_RATIO)); //Smallest angle possible

//Speed
double speed_kp, speed_kd, speed_ki; //In the function PID_CONFIG() they are given initial values
double speed_err,speed_err_prev,speed_err_ki,speed_err_kd; //All these initialize in 0
double speed_uk; //Work variable, just saves the PID calculations as a working register
//It uses the same MotorSpeed value, but in this mode this variable doesn't change in the same way
double DesiredSpeed = 20; //Desired speed (rpm)
//rev/min= x (step/s) * (1rev/200step) * (60s/1min)

//This stores the rotational velocity of the shaft
double RotationSpeed = 0; 
double RotationSpeed_current = 0;//This is the current's cycle steps count
double RotationSpeed_previous = 0;//This is the previous' cycle steps count
int CycleCount = 0; //Counts the number of interruptions between each speed calculation

//Speed measurement low pass filter
float vFilt = 0;
float vPrev = 0;

//Switch Debounce
//no switches in the program

//Program Control and debbug
bool dummy = false;
//Time keeeping variables
long m_current,m_previous,u_current,u_previous,u_interval;
//Time interval in ms
const long m_intertal = 1000; 


//TIMER VARIABLES
const unsigned long interval = 10; //In ms
double interval_seg = interval/1000.0; //interval in seconds
const unsigned long duration = 0;   //How many cycles do you want, 0 for infinite
bool  Toggle=false;

//***********************************************************************************************
//Functions
//***********************************************************************************************

//TIMER2 Handler
void TimerHandler();

//TIMER1 Handler (if needed)

//Motor Control
void calibrate();   //Initial motor calibration
void PositionControl();
void SpeedControl();

//PID Controller
//Original PID function
//int PID(double kp, double kd, double ki, double ref, double motor_input); 
//Initial values for both controllers
void PID_CONFIG(void);
//PID algorithm function
int PID(double kp, double kd, double ki, double ref, double motor_input, double err_kd, double err_ki, double err_prev);


//Encoder
void check_rotary(void); //Encoder values check
void Encoder_config(void);
void EncoderInterrupt(void); //Encoder interrupt pin mode

//COMMS configuration & DaisyChain protocol
void CommsConfig(void);
void JSONCONFIG();
void JSONSEND(command_t device_command, uint8_t device_id, float data);


//***********************************************************************************************
//Main Code
//***********************************************************************************************

void setup()
{
  //Read EEPROM VALUES
  // EEPROM.write(EEPROM_ID_ADDRESS,1); for configuration purposes
  EEPROM_ID = EEPROM.read(EEPROM_ID_ADDRESS); //Read the device ID from EEPROM
  EEPROM.get(EEPROM_GEAR_RATIO_ADDRESS,EEPROM_GEAR_RATIO); //Update the gear ratio to the output  
  EEPROM_CONTROL_MODE = EEPROM.read(EEPROM_MODE_ADDRESS); //Read the control mode from eeprom  
  // put your setup code here, to run once:
  //Timer Config
  ITimer2.init();
  // interval (in ms) and duration (in milliseconds). Duration = 0 or not specified => run indefinitely
  if (ITimer2.setInterval(interval, TimerHandler, duration))
  {
    digitalWrite(LED2,HIGH);
  } else
  {
    digitalWrite(LED2,LOW);
  }
  //Comms config
  CommsConfig();
  //Calibration of zero position
  calibrate(); 
  //Initial values for position controller
  PID_CONFIG(); 
  //Configurations for the rotary encoder
  Encoder_config(); 
  //Debugging comms LED
  pinMode(LED, OUTPUT);
  
  //Stepper Config
  //Steper theoretical max speed
  stepperX.setMaxSpeed(4000);
  //Stepper theoretical max acceleration
  stepperX.setAcceleration(100);
  RotationSpeed_previous = (double) micros(); //The first display counter value
  //Serial.print("\nReady\n");

}

void loop()
{
  // put your main code here, to run repeatedly:
  // stepperX.move(MotorSpeed);
  // // stepperX.setSpeed(100);
  stepperX.run();

  // m_current = millis();
  // if ((m_current - m_previous)>=m_intertal)
  // {
  //   m_previous = m_current;    
  //   Serial.print("\n\nElapsed Time: ");
  //   Serial.print(u_interval);
  // }

  //if (digitalRead(hall_switch)==HIGH)
  if (Toggle)
  {
    digitalWrite(LED2,HIGH);
  } else
  {
    digitalWrite(LED2,LOW);
  }

  //**********************************************************************************
  //                             Command Switch Case
  //**********************************************************************************

    switch (command)
    {
    case CMD_NULL:
      //I don't know what this does, probably does nothing
      //Debbug code
      if (EEPROM_CONTROL_MODE==POSITION)
      {
        digitalWrite(LED, HIGH);
      } else
      {
        digitalWrite(LED,LOW);
      }
      break;

    case CHANGE_ID:
      //Convert payload to int and write it to EEPROM address 0      
      EEPROM.write(EEPROM_ID_ADDRESS ,(uint8_t)payload);
      EEPROM_ID = EEPROM.read(EEPROM_ID_ADDRESS); //Updates to newest ID
      //Change command so that it does this only once
      command = CMD_NULL;
      break;

    case CONTROL_MODE:
      //Convert payload to int and write it to EEPROM address
      EEPROM.write(EEPROM_MODE_ADDRESS, (uint8_t)payload);
      EEPROM_CONTROL_MODE = EEPROM.read(EEPROM_MODE_ADDRESS); //Updates control mode
      //Change command so that it does this only once
      command = CMD_NULL;
      break;

    case WRITE_SETPOINT:
      //Depending on the control mode it enables the position PID or the speed PID
      if (EEPROM_CONTROL_MODE==POSITION)
      {
        //For position control, this value is in degrees
        Motorposition = (double)payload; //Desired position (degrees)
      } else 
      {
        //For speed control this value is in RPM
        DesiredSpeed = (double)payload; //Desired speed (rpm)
      }
      //Change command so that it does this only once
      command = CMD_NULL;
      break;   

    case READ_POSITION:
      //Set up the JSON package and send the current position as payload
      angle=displaycounter*(PositionControlResolution);
      JSONSEND(READ_POSITION, EEPROM_ID, angle);
      //Change command so that it does this only once
      command = CMD_NULL;
      break;

    case READ_SPEED:
      //Set up the JSON package and send the current rotational speed (rpm)
      JSONSEND(READ_SPEED,EEPROM_ID,RotationSpeed);
      //Change command so that it does this only once
      command = CMD_NULL;
      break;

    case WRITE_KP:
      //Get the payload and copy it to kp
      if (EEPROM_CONTROL_MODE==POSITION)
      {
        kp = payload;
        EEPROM.put(EEPROM_KP_ADDRESS,payload);
      } else
      {
        speed_kp = payload;
        EEPROM.put(SPEED_EEPROM_KP_ADDRESS,payload);
      }
      
      //Change command so that it does this only once
      command = CMD_NULL;
      break;

    case WRITE_KI:
      //Get the payload and copy it to ki
      if (EEPROM_CONTROL_MODE==POSITION)
      {
        ki = payload;
        EEPROM.put(EEPROM_KI_ADDRESS,payload);
      } else
      {
        speed_ki = payload;
        EEPROM.put(SPEED_EEPROM_KI_ADDRESS,payload);
      }
      
      //Change command so that it does this only once
      command = CMD_NULL;
      break;

    case WRITE_KD:
      //Get the payload and copy it to kd
      if (EEPROM_CONTROL_MODE==POSITION)
      {
        kd = payload;
        EEPROM.put(EEPROM_KD_ADDRESS,payload);
      } else
      {
        speed_kd = payload;
        EEPROM.put(SPEED_EEPROM_KD_ADDRESS,payload);
      }
      
      //Change command so that it does this only once
      command = CMD_NULL;
      break;

    case READ_KP:
      //Sends Kp over serial
      if (EEPROM_CONTROL_MODE==POSITION)
      {
        JSONSEND(READ_KP,EEPROM_ID,kp);
      } else
      {
        JSONSEND(READ_KP,EEPROM_ID,speed_kp);
      }
      
      //Change command so that it does this only once
      command = CMD_NULL;
      break;

    case READ_KI:
      //Sends Ki over serial
      if (EEPROM_CONTROL_MODE==POSITION)
      {
        JSONSEND(READ_KI,EEPROM_ID,ki);
      } else
      {
        JSONSEND(READ_KI,EEPROM_ID,speed_ki);
      }
      //Change command so that it does this only once
      command = CMD_NULL;
      break;

    case READ_KD:
      //Sends Kd over serial
      if (EEPROM_CONTROL_MODE==POSITION)
      {
        JSONSEND(READ_KD,EEPROM_ID,kd);
      } else
      {
        JSONSEND(READ_KD,EEPROM_ID,speed_kd);
      }
      //Change command so that it does this only once
      command = CMD_NULL;
      break;

    case READ_GEAR_RATIO:
      //Sends this motor's gear ratio over serial      
      JSONSEND(READ_GEAR_RATIO,EEPROM_ID,EEPROM_GEAR_RATIO);      
      //Change command so that it does this only once
      command = CMD_NULL;
      break;
    case WRITE_GEAR_RATIO:
      //Writes a gear ratio to use in this motor's output
      EEPROM.put(EEPROM_GEAR_RATIO_ADDRESS,payload);
      EEPROM.get(EEPROM_GEAR_RATIO_ADDRESS,EEPROM_GEAR_RATIO);

    default:
      //Another command that is not defined
      //Do nothing
      break;
    }


  //**********************************************************************************


  
  //**********************************************************************************
  //                              Encoder Readings (now done in interrupt)
  //**********************************************************************************
  // // If enough time has passed check the rotary encoder
  // if ((millis() - TimeOfLastDebounce) > DelayofDebounce) {
    
  //   check_rotary();  // Rotary Encoder check routine below
    
  //   PreviousCLK=digitalRead(PinCLK);
  //   PreviousDATA=digitalRead(PinDT);
    
  //   TimeOfLastDebounce=millis();  // Set variable to current millis() timer
  // }
  //**********************************************************************************


  //**********************************************************************************
  //                       Motor Controller Test Position 
  //**********************************************************************************
  // MotorSpeed = PID(kp, kd, ki, Motorposition, displaycounter); //Convert displaycounter to degrees
  // if ((MotorSpeed<=1) | (MotorSSpeed>=-1)){
  //   MotorSpeed = 0;
  //   stepperX.stop();
  // }else {
  //   stepperX.setSpeed(MotorSpeed);
  //   stepperX.run();
  // }  
  //                              Put this in a function
  //**********************************************************************************

  //**********************************************************************************
  //                       Motor Controller Test Speed
  //**********************************************************************************


  //                              Put this in a function
  //**********************************************************************************


}





//***********************************************************************************************
/*
 *                              Timer Interruptions (sample time)
*/
//***********************************************************************************************
//INTERRUPT ROUTINE
void TimerHandler()
{
  /*
  1. Medir numero de pasos (pos = displaycounter)
  2. Actualizar posición en grados (pasar de la interrupción de pines a esta interrupción)
  3. Medir velocidad en pasos (pasos previa interrupción - pasos actuales)/(tiempo muestreo) = pasos/seg
  4. Convertir velocidad a rpm = pasos/seg * 1 rev/200 pasos * 60 seg/1 min
  5. Preguntar el tipo de control -> if (EEPROM_CONTROL_MODE)
  6. Actualizar control = ejecutar algoritmo
  7. Calcular la resolución del motor (360°)/(200 pasos * GEAR_RATIO) para adaptarlo a la caja reductora de la salida
  */
  // Doing something here inside ISR
  Toggle = !Toggle; //Invert toggle
  //digitalWrite(LED2, Toggle); //update the status to port

  //Speed measurement
  /*
  Every interval, capture the newest position and substract the previous position.
  This gives the distance that the shaft rotated during the interval in steps.
  This distance divided the interval duration in seconds, gives the rotational speed in steps/s.
  */
  RotationSpeed_current = displaycounter; //Current Pos Count
  RotationSpeed = (RotationSpeed_current-RotationSpeed_previous)/interval_seg; //Delta Pos / Delta time (sample time) = speed
  RotationSpeed_previous = RotationSpeed_current; //Update previous Pos Count
  //Low pass filter (25Hz cuttoff) based on previous research
  vFilt = 0.854*vFilt + 0.0728*RotationSpeed + 0.0728*vPrev;
  vPrev = RotationSpeed;

  //Convert desired position from degrees to steps
  double Motorposition_steps = round(Motorposition/(PositionControlResolution));
  //Json outputs steps in degrees, so before this you must convert the ref to steps

  //Convert desired speed from rpm to steps/s
  double DesiredSpeed_steps = round((DesiredSpeed*steps_per_revolution)/60); //rev/min * 200 steps/rev * 1min/60s

  if (EEPROM_CONTROL_MODE==POSITION)
  {
    
    //            ref(steps)           motor_input(steps)
    MotorSpeed = PID(kp,kd,ki,Motorposition_steps,displaycounter,err_kd,err_ki,err_prev);
    stepperX.move(MotorSpeed); //Update inmediately the target position
    //El controlador de posición ya está listo
  } else 
  {
    
    MotorSpeed = PID(speed_kp,speed_kd,speed_ki,DesiredSpeed_steps, vFilt,speed_err_kd,speed_err_ki,speed_err_prev);
    stepperX.setSpeed(MotorSpeed);
  }
  //stepperX.run(); //In case the main loop gets interrupted
  //Serial plotter values
  // Serial.print("\n");
  // Serial.print(vFilt,6);
  // Serial.print("\n");

  

}
//***********************************************************************************************
/*
 *                          Serial Communication Readings
*/
//***********************************************************************************************
void serialEvent()
{
  if (Serial.available()>0)
  {
    // u_previous = micros();
    //If there is data in the buffer, deserialize it directly from it and store it in Inputdoc
    DeserializationError err = deserializeJson(Inputdoc, Serial);

    if (err == DeserializationError::Ok) 
    {
      //If there are no errors then update the variables      
      id = Inputdoc["id"];
      
    } 
    else 
    {
  
      // Flush all bytes in the serial port buffer
      while (Serial.available() > 0)
        Serial.read(); //Sends the bytes to the working registers and will be overwritten later
    }
    
    if (id!=EEPROM_ID) // and if Inputdoc is not null TO AVOID RESETTING THE id Variable
    {
      //This is not the right device, so it takes the package and sends it to the next one
      serializeJson(Inputdoc,Serial);
      id = EEPROM_ID; //reset the ID
    } else
    {
      command = Inputdoc["command"];
      payload = Inputdoc["payload"];
    }

  }
  
}


//***********************************************************************************************
/*
 *                                Program Functions
*/
//***********************************************************************************************
//Communication protocol configuration
void CommsConfig(void)
{
  Serial.begin(CommsBaudrate);   //Default settings
  while (!Serial) continue; //Wait for the module to be ready
  JSONCONFIG(); //Create document with default values
}

//Create the JSON documents
void JSONCONFIG()
{
  
  //Add values in the input document
  Inputdoc["command"] = command;
  Inputdoc["id"] = id;
  Inputdoc["payload"] = payload;

  //Add values in the output document
  Outputdoc["command"] = command;
  Outputdoc["id"] = id;
  Outputdoc["payload"] = payload;

  // Generate the minified JSON and send it to the Serial port.
  //
  //serializeJson(doc, Serial);
  // The above line prints:
  // {"command":0,"id":0,"payload":48.756080}
  // {"command":0,"id":1,"payload":40}

}

//Update JSON fields and send it right away
void JSONSEND(command_t device_command, uint8_t device_id, float data)
{
  //Add values in the document
  Outputdoc["command"] = device_command;
  Outputdoc["id"] = device_id;
  Outputdoc["payload"] = data;
  //Send the document over EUSART
  serializeJson(Outputdoc, Serial);

  //PROTOCOL IDEA:
  //Master serializes object into a string
  //Sends string
  //Slave gets every byte and appends it into a string
  //Slave deserializes the string into an object
  //Slave analyzes the object and executes the command
    //If it is just information, Slave serializes object once more
    //Puts it into the buffer and sends it
  //With uart daisy chain should be easy, try it with uart daisy chain
  //The master needs to have at least 2 uarts so it can get user inputs
  //from the serial port and send them through the other module
}

//***********************************************************************************************
/*
 *                                Motor Control
*/
//***********************************************************************************************

//This function sets up the zero position in the stepper
void calibrate(){
  //Initial configuration
  stepperX.setCurrentPosition(0);
  stepperX.setMaxSpeed(1000); //steps/sec the max speed at 16MHz is 4000 steps/sec
  stepperX.setAcceleration(100); //steps/sec^2

  pinMode(hall_switch,INPUT_PULLUP);
  pinMode(A1, OUTPUT); digitalWrite(A1,LOW);
  pinMode(A2, OUTPUT); digitalWrite(A2,LOW);
  pinMode(A3, OUTPUT); digitalWrite(A3,LOW);
  //Calibration process
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
  displaycounter = 0; //Configure this encoder position as zero
//  stepperX.runSpeed(50);
}

//Encoder configuration
void Encoder_config(){
  // Put current pins state in variables
  PreviousCLK=digitalRead(PinCLK);
  PreviousDATA=digitalRead(PinDT);

  // Set the Switch pin to use Arduino PULLUP resistors
  pinMode(PinSW, INPUT); //Not necesary since the encoder has a pullup resistor
  pinMode(PinDT, INPUT);
  pinMode(PinCLK, INPUT);
  pinMode(LED2, OUTPUT); //DEBBUG LIGHT
  //Attach interrupt pins
  //attachInterrupt(digitalPinToInterrupt(PinDT), EncoderInterrupt, FALLING);
  //Use the CLK pin interrupt to count
  attachInterrupt(digitalPinToInterrupt(PinCLK), EncoderInterrupt, CHANGE);

  //Time measurement of lapses
  // u_current = micros();
  // u_interval = u_current - u_previous;
  // u_previous = u_current;
}

//This function measures the time difference between
void EncoderInterrupt(void)
{ 
    //This interprets the data from the encoder
    check_rotary();
    PreviousCLK=digitalRead(PinCLK);
    PreviousDATA=digitalRead(PinDT);
    
}

// Check if Rotary Encoder was moved
void check_rotary() {

 if ((PreviousCLK == 0) && (PreviousDATA == 1)) {
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 0)) {
      displaycounter++;
      RPMCount++;
      // angle=displaycounter*(360.0/steps_per_revolution);
     //Serial.println(displaycounter);
     //Serial.println(int(angle*(360.0/40.0)));
    }
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 1)) {
      displaycounter--;
      RPMCount--;
      // angle=displaycounter*(360.0/steps_per_revolution);
      //Serial.println(displaycounter);
      //Serial.println(int(angle*(360.0/40.0)));
    }
  }

if ((PreviousCLK == 1) && (PreviousDATA == 0)) {
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 1)) {
      displaycounter++;
      RPMCount++;
      // angle=displaycounter*(360.0/steps_per_revolution);
      //Serial.println(displaycounter);
      //Serial.println(int(angle*(360.0/40.0)));
    }
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 0)) {
      displaycounter--;
      RPMCount--;
      // angle=displaycounter*(360.0/steps_per_revolution);
     //Serial.println(displaycounter);
     //Serial.println(int(angle*(360.0/40.0)));
    }
  }

if ((PreviousCLK == 1) && (PreviousDATA == 1)) {
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 1)) {
      displaycounter++;
      RPMCount++;
      // angle=displaycounter*(360.0/steps_per_revolution);
      //Serial.println(displaycounter);
      //Serial.println(int(angle*(360.0/40.0)));
    }
    if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 0)) {
      displaycounter--;
      RPMCount--;
      // angle=displaycounter*(360.0/steps_per_revolution);
     //Serial.println(displaycounter);
     //Serial.println(int(angle*(360.0/40.0)));
    }
  }  

if ((PreviousCLK == 0) && (PreviousDATA == 0)) {
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 0)) {
      displaycounter++;
      RPMCount++;
      // angle=displaycounter*(360.0/steps_per_revolution);
      //Serial.println(displaycounter);
      //Serial.println(int(angle*(360.0/40.0)));
    }
    if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 1)) {
      displaycounter--;
      RPMCount--;
      // angle=displaycounter*(360.0/steps_per_revolution);
      //Serial.println(displaycounter);  
      //Serial.println(int(angle*(360.0/40.0)));  
    }
  }
    
  }



//###########################################################
//          POSITION PID CONTROLLER FUNCTION
//###########################################################

int PID(double kp, double kd, double ki, double ref, double motor_input, double err_kd, double err_ki, double err_prev){
    double err = ref - motor_input;
    err_kd = err - err_prev;
    err_ki = err + err_ki;
    uk = kp*err + kd*err_kd + ki*err_ki;
    //update the previous error
    err_prev = err;


    //convert uk to integrer or use map function to output a coherent speed/position result.
    int result = (int) uk;
    // if ((result>=500))
    // {
    //   result = 500;
    // }
    // if ((result<=-500))
    // {
    //   result = -500;
    // }
    return result;
    //Try using the motor speed to control the motor and if it stops at a speed of steps/sec
    //then that is where this controller goes.
}

void PID_CONFIG(void){
  /*
          Remember to load these from EEPROM
  */
  //Position PID initial values
  //initial values of constants
  // kp = 1.88, ki = 0, kd = 0;
  EEPROM.get(EEPROM_KP_ADDRESS,kp); EEPROM.get(EEPROM_KI_ADDRESS,ki); EEPROM.get(EEPROM_KD_ADDRESS,kd);
  //initial values of error
  err_prev = 0; err_kd = 0; err_ki = 0;

  //Speed PID intial values
  //initial values of constants
  EEPROM.get(SPEED_EEPROM_KP_ADDRESS,speed_kp); EEPROM.get(SPEED_EEPROM_KI_ADDRESS,speed_ki); EEPROM.get(SPEED_EEPROM_KD_ADDRESS,speed_kd);
  //speed_kp = 1; speed_ki = 0; speed_kd = 0;
  //initial values of error
  speed_err_prev = 0; speed_err_kd = 0; speed_err_ki = 0;
}


//***********************************************************************************************
//                                Initial EEPROM VALUES
//***********************************************************************************************


/*
Usar micros()
Medir cuánto se tarda el JSON en serializar y deserializar
Medir cuánto se tarda en ir y regresar el dato
Se desea por lo menos una frecuencia de 10Hz de actualización de esclavos
Probar solo con UART
https://www.youtube.com/watch?v=zcb86TRxTxc

Tiempo comunicación = timepo en serializar + tiempo en deserializar + tiempo en enviar
                      1180 us (115200 b/s)    3488 us (115200b/s)     (1666.7 us a 115200b/s)
                      1048 us (1Mb/s)         220 us (1Mb/s)          (192 us a 1Mb/s)
                      1220 us (0.5Mb/s)       936 us (0.5Mb/s)        (384 us a 0.5Mb/s)
Tiempo comunicación (115200b/s) = 4876.33 us (en total por recibir y enviar)   
Tiempo comunicación fallida (115200b/s) = 4632 us (Recibe dato y lo envía al siguiente)                         
frecuencia comunicacion = 1/(Tiemop comunicación) = 205.07225720 Hz
frecuencia mínima = 10 Hz
frecuencia comunicación / frecuencia mínima = No. Dispositivos máximos
Maximos dispositivos = 20.50722572 = 20 dispositivos (a 115200 Bit/s)

Cálculos a 1Mb/s (es un poco inestable)
Tiempo comunicación = 1460 us (en total por recibir y enviar)   
Tiempo comunicación fallida = 288 us (Recibe dato y lo envía al siguiente)                         
frecuencia comunicacion = 1/(Tiemop comunicación) = 684.9315 Hz
frecuencia mínima = 10 Hz
frecuencia comunicación / frecuencia mínima = No. Dispositivos máximos
Maximos dispositivos = 68.493150 = 68 dispositivos

Cálculos a 500kb/s (0.5Mb/s)
Tiempo comunicación = 2540 us (en total por recibir y enviar)   
Tiempo comunicación fallida = 2096 us (Recibe dato y lo envía al siguiente)                         
frecuencia comunicacion = 1/(Tiemop comunicación) = 393.70078 Hz
frecuencia mínima = 10 Hz
frecuencia comunicación / frecuencia mínima = No. Dispositivos máximos
Maximos dispositivos = 39.370078 = 39 dispositivos

La frecuencia de 0.5Mb/s fue más estable que la de 1Mb/s
Datasheet pg. 165
*/



// // Check if Rotary Encoder was moved
// void check_rotary() {

//  if ((PreviousCLK == 0) && (PreviousDATA == 1)) {
//     if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 0)) {
//       displaycounter++;
//     }
//     if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 1)) {
//       displaycounter--;
//     }
//   }

// if ((PreviousCLK == 1) && (PreviousDATA == 0)) {
//     if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 1)) {
//       displaycounter++;
//     }
//     if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 0)) {
//       displaycounter--;
//     }
//   }

// if ((PreviousCLK == 1) && (PreviousDATA == 1)) {
//     if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 1)) {
//       displaycounter++;
//     }
//     if ((digitalRead(PinCLK) == 0) && (digitalRead(PinDT) == 0)) {
//       displaycounter--;
//     }
//   }  

// if ((PreviousCLK == 0) && (PreviousDATA == 0)) {
//     if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 0)) {
//       displaycounter++;
//     }
//     if ((digitalRead(PinCLK) == 1) && (digitalRead(PinDT) == 1)) {
//       displaycounter--;
//     }
//   }
    
//   }