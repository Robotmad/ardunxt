/* ArduNXT - Arduino based Lego Mondstorms NXT universal Remote Control Interface */
/* Please see http://code.google.com/p/ardunxt/ for further details and the latest version */
/* By Christopher Barnes */
 
/*
// Aspects of this software were derived from code used in ArduPilot
// By Chris Anderson, Jordi Munoz, Bill Premerlani, HappyKillMore
// James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister
// & Remzibi.
*/

// Standard Arduino Serial output is unbuffered so be careful about the impact that lots of diagnostics has on performance.
// You might like to try an alternative version of the HardwareSerial.h and HardwareSerial.cpp, by Kiril available from:
// Arduino Forum > Software > Development, at
// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?board=dev
// Search for "Improved HardwareSerial" and use these new files to replace those in hardware/arduino/cores/arduino 
// There should be enough memory on the ATMEGA328 to increase the buffer sizes too.  These are defined in HardwareSerial.h:
// #define USART_RX_BUFFER_SIZE  (64)
// #derine USART_TX_BUFFER_SIZE  (64)


//TODO
// Commands from NXT:
//  set RC Input centre
//  save config
//  control Mux

// Single status byte for NXT to read to say if anything has changed (bitmap of what?)

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <util/twi.h>

// Fucntion Prototypes for integration with twi4nxt code
extern void twi4nxt_attachSlaveRxEvent( void (*)(byte*, uint8_t) );
extern void twi4nxt_attachSlaveTxEvent( void (*)(void) );

#define TITLE_STRING    "ArduNXT Universal RC Interface"
#define VERSION_STRING  "V1.09"            // This is NOT the same as the version reported over the NXT interface 
                                           // which is defined in NXTI2C.pde

// Select the Baud rate you want to use:
#define SERIAL_BAUD		(115200)		   // Baud Rate used for DSM2 satellite receiver (and diagnostics)	
//#define SERIAL_BAUD     (38400)            // Baud Rate used for GPS and Diagnostics


#define NUM_RCI_CH			(7U)			// Number of RCInput Channels (e.g. direct from DSM2 satellite receiver)
#define NUM_SERVO_CH		(4U)			// total number of servo output channels


//#define ATTINY_IN_USE                    // If you do not hold the ATTiny in reset, on the Ardupilot prototype, then it is in control of the multiplexer
#define MINDSENSORS_NXT_SERVO_COMPATIBLE   // If you want FULL compatibility with Mindsensors NXT Servo Sensor (I2C address and Servo Position Readback)


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


// Flags to configure ArduNXT
typedef union {
  struct {
    unsigned bDSM2Enable:1;               // Serial Port used for DSM2 satellite receiver
  };
  struct {
    UINT_8 u8Value;
  };
} ConfigurationFlags;


// Assorted Flags 
typedef union {
  struct {
    unsigned bFrameUpdate:1;              // PWM Servo Output Frame Update
    unsigned bFrameMissed:1;              // PWM Servo Output Frame has been missed - check Diagnostics for loop execution performance...
  };
  struct {
    UINT_8 u8Value;
  };
} MiscFlags;


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
    unsigned bPresent:1;        // GPS module detected
  };
  struct {
    UINT_8 u8Value;
  };
} GPSMsgFlags;


// Definition of a GPS Record
typedef struct
{
//	UINT_16			u16Date;		// binary format date encoding	
	UINT_32			u32Time;		// Milliseconds since midnight (UTC)
	INT_32			i32Latitude;      	// Latitude (min/10000)
	INT_32			i32Longitude;      	// Longitude (min/10000)
	UINT_16			u16Altitude;    	// Altitude (m/10)
	UINT_16			u16Speed;		// Speed (knots/100) (or cm/s for UBX protocol)
	UINT_16			u16Heading;		// Heading (degrees/100)
} GPSRecord;


// Flags for Remote Control PWM Input Channels
typedef union {
  struct {
    // High level
    unsigned bValid:1;       // Flag to indicate that the channel is being received
    unsigned bUpdate:1;      // Flag to indicate that data has been updated  

    // Low level
    unsigned bRise:1;        // We have recorded a timestamp for rising edge
    unsigned bFall:1;        // We have recorded a timestamp for a falling edge
    unsigned bLost:1;        // We have lost a pulse measurements as the handler was not called frequently enough
    unsigned bTimerWrap:1;
    unsigned bWrap:1;        // Flag that we need to add PWM_PERIOD to fall time  
  };
  struct {
    UINT_8 u8Value;
  };
} 
RCInputFlags;


/*************************************************************************
 * 
 *************************************************************************/

// Where things are stored in EEPROM
#define EE_DIAGNOSTICS_FLAGS    (0x01)
#define EE_CONFIGURATION_FLAGS	(0x02)


/***************************************************************************
 General variables
 **************************************************************************/
GPSRecord						 m_GPSNew;
GPSMsgFlags	        			 g_GPSMsgFlags;
volatile RCInputFlags            g_RCIFlags[NUM_RCI_CH];	// RCInput channel flags
DiagnosticsFlags                 g_DiagnosticsFlags;
ConfigurationFlags				 g_ConfigurationFlags;	
volatile MiscFlags               g_MiscFlags;  
unsigned int					 g_u16Pulse[NUM_RCI_CH];		// RCInput pulse widths

/***************************************************************************

 **************************************************************************/
void setup()
{
  Init_ArduNXT();                  // Initialize application...
  Init_Diagnostics();              // Initialise diagnostics output
}


// Main loop executed as fast as possible
void loop()
{
  // Each "Handler" is designed to do a small amount of processing each time it is called in
  // a co-operative approach to multi-tasking.
  ServoOutput_Handler();
  RCInput_Handler();
  Analogue_Handler();
  if(g_ConfigurationFlags.bDSM2Enable)
  {
	  DSM2_Handler();
  }
  else
  {
  GPS_Handler();
  }	
  NXT_Handler();
  Multiplexer_Handler();
  
  // Support for development diagnostics and debugging information output
  if (g_DiagnosticsFlags.bDigitalInput) DigitalInput_Monitor();
  if (g_DiagnosticsFlags.bPerformance)  Diagnostics_Handler();
}


/*****************************************
 *****************************************/
void Init_ArduNXT(void)
{
  // Configure ATMega Hardware for ArduNXT
  // based on functionality of Ardupilot PCB
  // PD0 = RS232 Serial Data input
  // PD1 = RS232 Serial Data output
// Assignments which are commented out are initialised in the appropriate module...  
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
//  pinMode(12,OUTPUT);    // Blue LED output pin
//  pinMode(13,OUTPUT);    // Yellow LED output pin

  // Initialise basic variables
  g_DiagnosticsFlags.u8Value = 0U;
  g_MiscFlags.u8Value = 0U;
  
  // Initialise Serial Port
  Serial.begin(SERIAL_BAUD);  
  
  // Title Header
  Serial.println("");
  Serial.print(TITLE_STRING);
  Serial.println(VERSION_STRING);
  
  Load_Settings();//Loading saved settings
  
  // Override saved settings
  g_DiagnosticsFlags.bRCInput = TRUE;
  g_ConfigurationFlags.bDSM2Enable = TRUE;

  // Initialise all modules
  Save_Settings();
  Init_Multiplexer();
  Init_RCInputCh();
  Init_ServoOutput();
  Init_NXTIIC();
  if(g_ConfigurationFlags.bDSM2Enable)
  {
	  Init_DSM2();
  }
  else
  {
	  // If Serial Port is in use for DSM2 satellite receiver then it can't also do GPS
  Init_GPS();
  }

  // Everything initialised - enable interrupts
  sei();
}

/*****************************************************************************
 *****************************************************************************/
void Load_Settings(void)
{
  Serial.println("Load Settings");
  g_DiagnosticsFlags.u8Value =(byte)eeprom_read_byte((const uint8_t *)EE_DIAGNOSTICS_FLAGS); 
  g_ConfigurationFlags.u8Value =(byte)eeprom_read_byte((const uint8_t *)EE_CONFIGURATION_FLAGS);
}

void Save_Settings(void)
{
  Serial.println("Save Settings");
  eeprom_busy_wait(); 
  eeprom_write_byte((uint8_t *)EE_DIAGNOSTICS_FLAGS, g_DiagnosticsFlags.u8Value);
  eeprom_write_byte((uint8_t *)EE_CONFIGURATION_FLAGS, g_ConfigurationFlags.u8Value);
}



