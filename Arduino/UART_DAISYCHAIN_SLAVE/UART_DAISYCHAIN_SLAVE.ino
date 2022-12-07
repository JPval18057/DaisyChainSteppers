//###########################################################
/*
 * Author: Juan Pablo Valenzuela
 * Creation date:
 * Description:
 * This is the code of the daisy chain protocol communication
 */
 //###########################################################
 //Libraries
 #include <AccelStepper.h> //for stepper motor control with DRV8825
 #include <MultiStepper.h>  //probably not going to be useful since each
 //arduino only has 1 stepper
  #include <EEPROM.h> //for identification purposes mainly
 
 //###########################################################
 //Definitions
 //for ESP32 it is 2
#define LED LED_BUILTIN
//MOTOR CONTROL
#define dirPin 11
#define stepPin 12
#define motorInterfaceType 1
#define hall_switch 2  //10 is the default
#define LED 10
//COMMUNICATIONS
#define comandoConfig 'C'       // Para utilizar el listado de comandos de configuración.
#define comandoMovimiento 'M'   // Para utilizar el listado de comandos de movimiento.
#define comandoDiagnostico 'D'  // Para utilizar el listado de comandos de diagnóstico.

//###########################################################
//Objects
// Create a new instance of the AccelStepper class:
AccelStepper stepperX(motorInterfaceType, stepPin, dirPin);

//###########################################################
//Variables
//MOTOR CONTROL 
// Rotary Encoder Module connections
const int PinSW=9;   // Rotary Encoder Switch
const int PinDT=8;    // DATA signal
const int PinCLK=7;    // CLOCK signal

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

void check_rotary(void);
void Encoder_config(void);

//boolean debounceButton(boolean state);
void checkswitch(void);

//Communication variables
const byte datalength = 128;
char receivedData [datalength];
char tempChars [datalength];

char inData;
char index = 0;
static boolean receiving = false;
bool ledstate=false;
char start = '<';
char finish = '>';
char split = ';';
boolean Newdata = false;
int constSiguientes = 0;

bool calibration;
char addressMotor  = 0;

//PID Controller variables
double kp, kd, ki; //si no se permiten inicializar en valores constantes ponerle los valores en el setup como PID_CONFIG();
double err,err_prev,err_ki,err_kd; ///verificar que se inicializen en 0
double uk; //Work variable, just saves the PID calculations as a working register
int MotorSpeed = 100;
double Motorposition = 180;


//Program control variables


//###########################################################
//Functions
void coms_config(); //UART mainly
void calibrate();   //Initial motor calibration
//POSITION CONTROLLER
//prototype
int PID(double kp, double kd, double ki, double ref, double pos_motor);
void PID_CONFIG(void);

//###########################################################
//						Main code
void setup() {
  // put your setup code here, to run once:
  coms_config(); //Communication config
  // LED to show state of hall sensor
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  calibrate(); //calibration of zero position
  PID_CONFIG(); //Initial values for position controller
}

void loop() {
  // put your main code here, to run repeatedly:

  //ENCODER CODE
  // If enough time has passed check the rotary encoder
  if ((millis() - TimeOfLastDebounce) > DelayofDebounce) {
    
    check_rotary();  // Rotary Encoder check routine below
    
    PreviousCLK=digitalRead(PinCLK);
    PreviousDATA=digitalRead(PinDT);
    
    TimeOfLastDebounce=millis();  // Set variable to current millis() timer
  }
  checkswitch(); //check if button was pressed (in this application it is never pressed)
  
  //MOTOR CODE
  if (digitalRead(hall_switch)==HIGH){
    digitalWrite(LED, LOW);
  } else {
    digitalWrite(LED, HIGH);
  }
  //COMS

  //MOTOR TEST
  MotorSpeed = PID(kp, kd, ki, Motorposition, displaycounter);
  if (MotorSpeed<1){
    MotorSpeed = 0;
    stepperX.stop();
  }else {
    stepperX.setSpeed(MotorSpeed);
    stepperX.run();
  }  
}


//###########################################################
//					User defined functions
//###########################################################
//Daisy chain configuration base
void coms_config(){
  Serial.begin(115200,SERIAL_8N1);
  while(!Serial){
    //Wait for the module to get ready
  }

}

//This reads the incoming data
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    inData = Serial.read();
    if (inData=='A'){
      addressMotor=Serial.read();
      comandoAsignarAddr(0,addressMotor);
    }
  }
}

//This function sets up the zero position in the stepper
void calibrate(){
  //Initial configuration
  stepperX.setCurrentPosition(0);
  stepperX.setMaxSpeed(1000); //steps/sec the max speed at 16MHz is 4000 steps/sec
  stepperX.setAcceleration(100); //steps/sec^2

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

void comandoAsignarAddr(char addrPos, char addr){
    addressMotor = addr;
    EEPROM.write(addrPos,addressMotor);
    addressMotor = addressMotor + 1;
    Serial.write("A");
    Serial.write(addressMotor);
}

/*
 * ENCODER FUNCTIONS
 */

 void Encoder_config(){
  // Put current pins state in variables
  PreviousCLK=digitalRead(PinCLK);
  PreviousDATA=digitalRead(PinDT);

  // Set the Switch pin to use Arduino PULLUP resistors
  pinMode(PinSW, INPUT_PULLUP); //Not necesary since the encoder has a pullup resistor
  pinMode(PinDT, INPUT);
  pinMode(PinCLK, INPUT);
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

//###########################################################
//          POSITION PID CONTROLLER FUNCTION
//###########################################################

int PID(double kp, double kd, double ki, double ref, double pos_motor){
    err = ref - pos_motor;
    err_kd = err - err_prev;
    err_ki = err + err_ki;
    uk = kp*err + kd*err_kd + ki*err_ki;
    //update the previous error
    err_prev = err;
    
    //convert uk to integrer or use map function to output a coherent speed/position result.
    int result = (int) uk;
    return result;
    //Try using the motor speed to control the motor and if it stops at a speed of steps/sec
    //then that is where this controller goes.
}

void PID_CONFIG(void){
  //initial values of constants
  kp = 1; ki = 0; kd = 0;
  //initial values of error
  err_prev = 0; err_kd = 0; err_ki = 0;
}
