/* ArduNXT - Arduino based Lego Mondstorms NXT universal Remote Control Interface */
/* Please see http://code.google.com/p/ardunxt/ for further details and the latest version */
/* By Christopher Barnes */
/* 
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

// WARNING - at present this code requires a modified version of WIRE and TWI 
// I need to find out how to get this version used by default or replace them with new code.

// Standard Arduino Serial output is unbuffered so be careful about the impact that lots of diagnostics has on performance.
// You might like to try an alternative version of the HardwareSerial.h and HardwareSerial.cpp, by Kiril available from:
// Arduino Forum > Software > Development, at
// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?board=dev
// Search for "Improved HardwareSerial" and use these new files to replace those in hardware/arduino/cores/arduino 
// There should be enough memory on the ATMEGA328 to increase the buffer sizes too.  These are defined in HardwareSerial.h:
// #define USART_RX_BUFFER_SIZE  (64)
// #derine USART_TX_BUFFER_SIZE  (64)


//TODO
// consider RC working - in control of MUX/Mode, no RC input - need to take control of PWM outputs
// Commands from NXT:
//  set centre
//  save config
//  control Mux
// Single status byte for NXT to read to say if anything has changed (bitmap of what?)
// Double buffer GPS data so that you can read it all without risk of partial update...

#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/io.h>

#define TITLE_STRING    "ArduNXT Universal RC Interface"
#define VERSION_STRING  " V1.03"
#define SERIAL_BAUD     (57600)



// Some of the code has been migrated from a previous project which uses the following type definitions:
typedef signed char   INT_8;
typedef unsigned char UINT_8;
typedef signed int    INT_16;
typedef unsigned int  UINT_16;
typedef unsigned long UINT_24;  // Not sure that there is any way to get a true 24 bit type in Arduino?
typedef signed long   INT_32;
typedef unsigned long UINT_32;

#ifndef TRUE
#define TRUE          (0x01)
#endif
#ifndef FALSE
#define FALSE         (0x00)
#endif


// Flags to control which Diagnostics outputs we want to see on the serial port
typedef union {
  struct {
    unsigned bGPS:1;                // GPS decoding
    unsigned bRCInput:1;            // RC Input
    unsigned bServoOutput:1;        // Servo Output
    unsigned bNXTInterface:1;       // I2C NXT interface
    unsigned bMultiplexer:1;        // Multiplexer state
    unsigned bDigitalInput:1;       // Digital Inputs 
    unsigned bDigitalOutput:1;      // Digital Outputs
    unsigned bPerformance:1;        // Software performance metrics
  };
  struct {
    UINT_8 u8Value;
  };
} DiagnosticsFlags;


// Flags to indicate which aspects of the GPS data have been received
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
GPSRecord	        		 m_GPSNew;
GPSMsgFlags	        		 g_GPSMsgFlags;
DiagnosticsFlags                         g_DiagnosticsFlags;


/***************************************************************************

 **************************************************************************/
void setup()
{
  Init_NXTUniversalRCInterface();  // Initialize application...
  Init_Diagnostics();              // Initialise diagnostics output
}


// Main loop executed as fast as possible
void loop()                        //Main Loop
{
  // Each "Handler" is designed to do a small amount of processing each time it is called in
  // a co-operative approach to multi-tasking.
  RCInput_Handler();
//Analogue_Handler();
  GPS_Handler();
  NXT_Handler();
  Multiplexer_Handler();
  
  // Support for development diagnostics and debugging information output
  if (g_DiagnosticsFlags.bDigitalInput) DigitalInput_Monitor();
  if (g_DiagnosticsFlags.bPerformance)  Diagnostics_Handler();
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
//  pinMode(4,INPUT);      // MUX output
//  pinMode(5,INPUT);      // Servo output pin 3
//  pinMode(6,INPUT);      // RC Input pin 2
//  pinMode(7,OUTPUT);     // GPS RX Input Mux output can we afford to spare a pin for this?
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
  g_DiagnosticsFlags.u8Value = 0xFB;  // Turn on all diagnostics of interest
  
  Save_Settings();
  Init_Multiplexer();
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

