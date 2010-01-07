#include <Wire.h>
// WARNING - at present this code requires a modified version of WIRE and TWI 
// I need to find out how to get this version used by default or replace them with new code.
// Serial output is unbuffered so be careful about the impact that lots of diagnostics has on performance.

/*
// Aspects of this software were derived from code used in ArduPilot
   By Chris Anderson, Jordi Munoz
   Special thanks to: 
  -Bill Premerlani
  -HappyKillMore
  -James Cohen.
  -JB from rotorFX.
  -Automatik.
  -Fefenin
  -Peter Meister
  -Remzibi
*/


//TODO
// DONE Get PWM in and out to use a common range
// consider RC working - in control of MUX/Mode, no RC input - need to take control of PWM outputs
// eliminate need for ATTiny
// DONE increase number of PWMin and out (coarse for Failsafe input?)
// Commands from NXT:
//  set centre
//  save config
//  control Mux
// Single status byte for NXT to read to say if anything has changed (bitmap of what?)
// Double buffer GPS data so that you can read it all without risk of partial update...

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/io.h>

//#define Start_Byte 0x0E //Used to read the EEPROM
#define TITLE_STRING  "ArduNXT Universal RC Interface"
#define VERSION_STRING  " V1.02"
#define SERIAL_BAUD  (57600)

// Some of the code has been migrated from a previous project which uses the following type definitions:
typedef signed char   INT_8;
typedef unsigned char UINT_8;
typedef unsigned int  UINT_16;
typedef unsigned long UINT_24;  // Not sure that there is any way to get a true 24 bit type in Arduino?
typedef unsigned long UINT_32;
typedef signed long   INT_32;

#define TRUE          (0x01)
#define FALSE         (0x00)


// Flags to control which Diagnostics outputs we want to see on the serial port
typedef union {
  struct {
    unsigned bGPS:1;
    unsigned bRCInput:1;
    unsigned bServoOutput:1;
  };
  struct {
    UINT_8 u8Value;
  };
} DiagnosticsFlags;


// Flags to indicate which aspects of the GPS Record have been received
typedef union {
  struct {
//  unsigned bDate:1;  
    unsigned bTime:1;           // Time
    unsigned bLatLon:1;         // Latitude and Longitude 
    unsigned bAltitude:1;       // Altitude 
    unsigned bSpeed:1;		// Speed and Heding
    unsigned bValid:1;          // GPS Fix Status (i.e. 2D or better)
    unsigned bUpdate:1;         // Flag to indicate that data has been updated (i.e. a full set of new values)
  };
  struct {
    UINT_8 u8Value;
  };
} GPSMsgFlags;

// Definition of a GPS Record
typedef struct
{
//	UINT_16			u16Date;		// binary format date encoding	
	UINT_24			u24Time;		// Seconds since midnight (UTC))
	UINT_8			u8CentiSeconds;	        // CentiSeconds	
//	UINT_32			u32Time;		// Milliseconds TOW
//	UINT_32			u32DateTime;      	// combined date & time
	INT_32			i32Latitude;      	// Latitude (min/10000)
	INT_32			i32Longitude;      	// Longitude (min/10000)
	UINT_16			u16Altitude;    	// Altitude (m/10)
	UINT_16			u16Speed;		// Speed (knots/100) (or cm/s for UBX protocol)
	UINT_16			u16Heading;		// Heading (degrees/100)
} GPSRecord;


// Where things are stored in EEPROM
#define EE_DIAGNOSTICS_FLAGS    (0x01)



/***************************************************************************
 General variables
 **************************************************************************/
/*
float analog0=511; //Roll Analog  
float analog1=511; //Pitch Analog
float analog2=511; //Z Analog
float analog3=511; //I have to change this name is the airspeed sensor... SOON! 
unsigned int rx_Ch[2];
*/

GPSRecord	        		 m_GPSNew;
GPSMsgFlags	        		 g_GPSMsgFlags;
DiagnosticsFlags                         g_DiagnosticsFlags;

/***************************************************************************

 **************************************************************************/
void setup()
{
  Init_NXTUniversalRCInterface();  //Initialize application...
}

void loop()                        //Main Loop
{
//  unsigned long u32time;

//  u32time = millis();
//  Serial.print("C");
  RCInput_Handler();
//  Serial.print(millis()-u32time);

//  u32time = millis();
//  Serial.print(" A");
//  catch_analogs(); //Reads and average the sensors when is doing nothing... Function localted in "Sensors" tab.
//  Serial.print(millis()-u32time);

//  u32time = millis();
//  Serial.print(" G");
  GPS_Handler();
//  Serial.print(millis()-u32time);

//  u32time = millis();
//  Serial.print(" X");
  NXTHandler();
//  Serial.println(millis()-u32time);

  if (g_DiagnosticsFlags.bRCInput) RCInput_Monitor();
}



/*****************************************
 *****************************************/
void Init_NXTUniversalRCInterface(void)
{
  // Configure ATMega Hardware for NXTUniversalRCInterface
  // based on functionality of Ardupilot PCB
  // PD0 = RS232 Serial Data input
  // PD1 = RS232 Serial Data output
//  pinMode(2,INPUT);      // RC Input pin 0
//  pinMode(3,INPUT);      // RC Input pin 1
  pinMode(4,INPUT);      // MUX input pin from ATTiny [to be dropped in final hardware]
  pinMode(5,INPUT);      // Mode input pin from ATTiny [to be dropped in final hardware]
//  pinMode(6,INPUT);      // RC Input pin 2
// Servo outputs are configured as such within the ServoOutput module  
//  pinMode(7,OUTPUT);     // Servo output pin 3
//  pinMode(8,OUTPUT);     // Servo output pin 2 
//  pinMode(9,OUTPUT);     // Servo output pin 0
//  pinMode(10,OUTPUT);    // Servo output pin 1
//  pinMode(11,INPUT);     // RC Input pin 3
  pinMode(12,OUTPUT);    // Blue LED output pin
  pinMode(13,OUTPUT);    // Yellow LED output pin

  // Initialise basic variables
  g_DiagnosticsFlags.u8Value = 0U;

  // Initialise Serial Port
  Serial.begin(SERIAL_BAUD);  
  
  // Title Header
  Serial.println("");
  Serial.print(TITLE_STRING);
  Serial.println(VERSION_STRING);
  
  Load_Settings();//Loading saved settings
  
  //TEMP
  g_DiagnosticsFlags.u8Value = 0x03;  // Turn on all diagnostics of interest
  
  Save_Settings();
  
  Init_RCInputCh();
  Init_ServoOutput();
  Init_NXTIIC();
  Init_GPS();
}

/*****************************************************************************
 *****************************************************************************/
void Load_Settings(void)
{
  Serial.println("Load Settings");
  g_DiagnosticsFlags.u8Value =(byte)eeprom_read_byte((const uint8_t *)EE_DIAGNOSTICS_FLAGS); 
}

void Save_Settings(void)
{
  Serial.println("Save Settings");
  eeprom_busy_wait(); 
  eeprom_write_byte((uint8_t *)EE_DIAGNOSTICS_FLAGS, g_DiagnosticsFlags.u8Value);
}


