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
 //#include <MultiStepper.h>  probably not going to be useful since each
 //arduino only has 1 stepper
  #include <EEPROM.h> //for identification purposes mainly
 
 //###########################################################
 //Definitions
 //for ESP32 it is 2
#define LED 2

#define comandoConfig 'C'       // Para utilizar el listado de comandos de configuración.
#define comandoMovimiento 'M'   // Para utilizar el listado de comandos de movimiento.
#define comandoDiagnostico 'D'  // Para utilizar el listado de comandos de diagnóstico.

//###########################################################
//Objects


//###########################################################
//Variables
//Communication variables
const byte datalength = 128;
char receivedData [datalength];
char tempChars [datalength];

char inData;
//char index;   this is a name used in the library probably
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
int kp = 0;
int ki = 0;
int kd = 0;

//Program control variables
long current,previous;
const int interval = 1000;
char uart0_Data;

//###########################################################
//Functions
void coms_config(); //UART mainly
void calibrate();   //Initial motor calibration
void PID();         //PID controller (probably going to return the values)

//###########################################################
//            Main code
void setup() {
  // put your setup code here, to run once:
  coms_config();
  //Indicator LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  current = millis();
  if ((current-previous)>=interval){
    Serial2.write("0A");
    previous=current;
  }
}


//###########################################################
//          User defined functions
//###########################################################
//Daisy chain configuration base
void coms_config(){
  Serial2.begin(115200,SERIAL_8N1);
  while(!Serial2){
    //Wait for the module to get ready
  }
  Serial.begin(15200);
  while (!Serial){
    //Wait for serial 0 to be ready
  }
  Serial.print("\nSerial 0 ready!\n");
  calibrate();

}

//This reads the incoming data
/*
 * There are multiple interrupt routines
 * serialEvent, serialEvent1, serialEvent2, serialEvent3
 * This depends on the number of uart modules
 */
void serialEvent2() {
  while (Serial2.available()) {
    // get the new byte:
    inData = Serial2.read();
    if (inData==65){
      addressMotor=Serial2.read();
      Serial.print("\nMotores detectados: ");
      Serial.print(addressMotor);
    }
  }
}

//This is in case the ESP32 gets data from the PC
void serialEvent(){
  while(Serial.available()) {
    uart0_Data = Serial.read();
    if (uart0_Data=='C') {
      String mssg = "\nConnected devices: \n";
      Serial.print(mssg);
    }
  }
}

//This functions assigns a number to each microcontroller
void calibrate(){
  Serial2.write("0A");
}
