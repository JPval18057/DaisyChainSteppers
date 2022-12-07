//LIBRARIES
#include <Arduino.h>
#include <EEPROM.h>
//#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <Daisychain.h>
//#include <TimerInterrupt.h>
//Use EEPROM.put to put the float values
//You need 4 bytes for a float value and 2 for an integrer value
//Macros 
#define LED0 10
#define LED1 11
#define LED2 12
#define LED3 13
#define BUFFER_SIZE 128


//Variables
//JSON document
// Inside the brackets, 200 is the RAM allocated to this document.
// Don't forget to change this value to match your requirement.
// Use arduinojson.org/v6/assistant to compute the capacity.
StaticJsonDocument<BUFFER_SIZE> Inputdoc;
StaticJsonDocument<BUFFER_SIZE> Outputdoc;
//The assistant recommends 24 bytes but 50 should be enough

// StaticJsonObject allocates memory on the stack, it can be
// replaced by DynamicJsonDocument which allocates as needed.
//
// DynamicJsonDocument  doc(200);

//This is the variable that stores the command to send
command_t command = CMD_NULL;
// command_t command = WRITE_SETPOINT;
// command_t command = WRITE_KP;


//This is the device id
uint8_t id = 0;
uint8_t EEPROM_ID; 
//This is the payload mainly for PID constants
float payload = 1.0000;
//This variable stores the JSON string
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

//Time debug
long current,previous,elapsedTime,millis_current,millis_prev;
const long interval = 1000;
bool measureFinished = false;

//Functions
void JSONCONFIG();
void JSONSEND(command_t device_command, uint8_t device_id, float data);

void setup() {
  // put your setup code here, to run once:
  EEPROM_ID = EEPROM.read(0); //Read the device ID from EEPROM
  Serial.begin(115200);
  while (!Serial) continue; //Wait for the module to be ready
  //JSON document setup (package)
  JSONCONFIG();
  pinMode(13,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  
}

//Program functions
// void serialEvent()
// {
//   previous = micros();
//   if (Serial.available())
//   {
//     //If there is data in the buffer, deserialize it directly from it and store it in Inputdoc
//     DeserializationError err = deserializeJson(Inputdoc, Serial);

//     if (err == DeserializationError::Ok) 
//     {
//       //If there are no errors then update the variables      
//       id = Inputdoc["id"];
      
//     } 
//     else 
//     { 
//       //There was an error so we erase the buffer
//       // Serial.print("deserializeJson() returned ");
//       // Serial.println(err.c_str());
  
//       // Flush all bytes in the serial port buffer
//       while (Serial.available() > 0)
//         Serial.read(); //Sends the bytes to the working registers and will be overwritten later
//     }

//     if (id!=EEPROM_ID)
//     {
//       //This is not the right device, so it takes the package and sends it to the next one
//       serializeJson(Inputdoc,Serial);
//       id = EEPROM_ID; //reset the ID
//     } else
//     {
//       command = Inputdoc["command"];
//       payload = Inputdoc["payload"];
//       measureFinished = (bool) payload;      
//     }

//     current = micros();
//     elapsedTime = current - previous;    

//   }
  
  
// }


//Interrupt functions
//In this function we create and send a JSON document through EUSART
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



}

void JSONSEND(command_t device_command, uint8_t device_id, float data)
{
  previous = millis();
  //Add values in the document
  Outputdoc["command"] = device_command;
  Outputdoc["id"] = device_id;
  Outputdoc["payload"] = data;
  //Send the document over EUSART
  //serializeJson(doc, Serial);
  //Save the document in a String variable
  String package;
  serializeJson(Outputdoc, package);
  //Mostramos el texto en el monitor serial
  Serial.print(package);
  current = millis();
  elapsedTime = current - previous;
  //Thi string can be sent easily via SPI, EUSART, HTTP or many others

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
