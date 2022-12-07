/*
  This is the I2C SLAVE COMMUNICATION PROTOCOL
  TEST
  Board: arduino nano original
*/
//Macros 
//Built in LED is 13 on atmega328
#define LED 13
//EUSART Universal Baudrate for this project
#define CommsBaudrate 115200 //1,000,000
//Is 64 bytes by default but the JSON Assistant recommends 24 bytes for serializing and 64 for deserializing
#define BUFFER_SIZE 64       

//Libraries
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

//JSON DOCUMENT
//Document to store incoming information
StaticJsonDocument<BUFFER_SIZE> doc;
StaticJsonDocument<BUFFER_SIZE> InputDoc;
StaticJsonDocument<BUFFER_SIZE> OutputDoc;

//Variables
//EEPROM
uint8_t FLOAT_SIZE = 4;
uint8_t EEPROM_ID;
uint8_t EEPROM_ID_ADDRESS = 0;

//Daisychain
String response;
String InData, OutData;
double payload;
uint8_t id;
uint8_t command;

//Time variables
long current,previous;
const long interval = 1000;


//Functions
void Daisychain_begin();
void Daisychain_transmit(int address, int data);
void MasterRequest();
void SlaveReceive(int BytesWritten);
void JSONCONFIG();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(CommsBaudrate);
  while (!Serial)
  {
    //Wait for the module to start up
  }
  
  //Read EEPROM VALUES
  EEPROM_ID = EEPROM.read(EEPROM_ID_ADDRESS); //Read the device ID from EEPROM

  
  //Begin as Slave
  Wire.begin(EEPROM_ID);
  //Function to run when master receives data
  Wire.onReceive(SlaveReceive);
  //Function to run when master requests data
  Wire.onRequest(MasterRequest);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  current = millis();
  if ((current - previous)>interval)
  {
    previous = current;

  }

}


void MasterRequest() 
{
  //Insert switch case to interpret what the master wants (only for readings)
  switch (command)
  {
  case 0:
  //Null command
    payload = 10;
    break;
  case 1:
  //Read kp
    payload = 1;
    break;
  case 2:
  //Read position
    payload = 90.0;
    break;
  default:
    payload = 0;
    break;
  }
  //Convert string in buffer to a document for sending
  OutputDoc["command"] = 0;
  OutputDoc["id"] = EEPROM_ID;
  OutputDoc["payload"] = payload;
  //Put a simple wire.write to write the serialized JSON
  serializeJson(OutputDoc, OutData);
  Wire.write(OutData); //look up for const char*
}

void SlaveReceive(int BytesWritten)
{
  while (Wire.available()>0)
  {
    char inChar = Wire.read();
    InData += inChar;
  }
  //Convert string in buffer to a document to read the variables
  deserializeJson(InputDoc,InData);
  //Extract the package id
  id = InputDoc["id"];
  //If the id is the same as the device's, then extract the whole package
  if (id==EEPROM_ID)
  {
    payload = InputDoc["payload"];
    command = InputDoc["command"];
  } //I am not sure if I need to send the data to the next one
  
}


void JSONCONFIG()
{
  //Test document
  doc["command"] = 0;
  doc["id"] = 2;
  doc["payload"] = 1.0;
  //Output document
  OutputDoc["command"] = 0;
  OutputDoc["id"] = 2;
  OutputDoc["payload"] = 1.0;
  //Input document
  InputDoc["command"] = 0;
  InputDoc["id"] = 2;
  InputDoc["payload"] = 1.0;
  serializeJson(doc, response);

  //Convert document into a string called "OutData"
  serializeJson(OutputDoc, OutData);
  serializeJson(InputDoc, InData);
}