#ifndef __DAISYCHAIN_H__
#define __DAISYCHAIN_H__


//First define the values for the datatype command_t
typedef enum
{
	CMD_NULL,
	CHANGE_ID,
	CONTROL_MODE,
	WRITE_SETPOINT,
	READ_POSITION,
	READ_SPEED,
	WRITE_GEAR_RATIO,
	WRITE_KP,
	WRITE_KI,
	WRITE_KD,
	READ_KP,
	READ_KI,
	READ_KD,
	READ_GEAR_RATIO
} command_t ;
//Could add a CALIBRATE cmd to auto enumerate the devices on power on

typedef enum
{
	POSITION,
	SPEED
} control_mode_t;

// Command list (outdated)
// No action - 0 (se usará cuando los motores esclavos le manden información al master)
// Assign ID - 1
// Set Control Mode - 2
// Write setpoint - 3
// Read position - 4
// Read velocity (speed + direction) - 5
// Write Kp - 6
// Write Ki - 7
// Write Kd - 8
// Read Kp - 9
// Read Ki - 10
// Read Kd - 11

//This is the basic structure of the package
//{"command": 0,"id": 1,"payload": 48.75608}
//{"command": 0,"id": 1,"payload": 40}
//The slaves ids range from 1 to 255, id:0 is reserved for master

//The JSON assistant recommends 24 bytes of memory to store it
//And 64 bytes to deserialize it
//Arduino has 64 bytes by default as a Serial Buffer
//We should be ok if we reserve 64 bytes for the JSON
/*
StaticJsonDocument<24> doc;

doc["command"] = 0;
doc["id"] = 1;
doc["payload"] = 48.75608;

serializeJson(doc, output);
*/

/*
To modify the serial buffer size in platformIO use this:

[env:uno]
platform = atmelavr
board = uno
framework = arduino
monitor_speed = 115200		;This sets up the Serial Port baud rate
build_flags = -D SERIAL_RX_BUFFER_SIZE=256

https://community.platformio.org/t/how-to-increase-the-arduino-serial-buffer-size/122/3

*/


#endif // __DAISYCHAIN_H__