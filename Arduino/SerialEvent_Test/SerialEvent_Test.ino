/*
  Serial Event example

  When new serial data arrives, this sketch adds it to a String.
  When a newline is received, the loop prints the string and clears it.

  A good test for this is to try it with a GPS receiver that sends out
  NMEA 0183 sentences.

  NOTE: The serialEvent() feature is not available on the Leonardo, Micro, or
  other ATmega32U4 based boards.

  created 9 May 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/SerialEvent
*/

#include <ArduinoJson.h>
#define BUFFER_SIZE 64

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

//Functions
StaticJsonDocument<BUFFER_SIZE> doc; //JSON DOCUMENT
StaticJsonDocument<BUFFER_SIZE> Outputdoc; //JSON DOCUMENT
String Serialized_doc;

void JSON_CONFIG();

void setup() {
  // initialize serial:
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(BUFFER_SIZE);
  JSON_CONFIG();
}

void loop() {
    // print the string when a newline arrives:
    delay(1); //Heavy load simulator
    if (stringComplete) {
      DeserializationError error = deserializeJson(Outputdoc, inputString);
  
    if (!error) {
      
      uint8_t command = Outputdoc["command"]; // 0
      uint8_t id = Outputdoc["id"]; // 1
      float payload = Outputdoc["payload"]; // 48.75608
      Serial.print("\nCommand:");
      Serial.print(command,6);
      Serial.print("\nID:");
      Serial.print(id,6);
      Serial.print("\nPayload:");
      Serial.print(payload,6);
    } else {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    
    
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()>0) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  
}

void JSON_CONFIG()
{
  doc["command"] = 0;
  doc["id"] = 1;
  doc["payload"] = 48.75608;
  
  serializeJson(doc, Serialized_doc);
  Serial.print(Serialized_doc); //send via UART0
}
