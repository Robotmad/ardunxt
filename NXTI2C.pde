/*************************************************************
 *  Arduino Lego Mindstorms NXT Interface
 *  Written by C M Barnes
 *  October 2009
 *
 *  Connect NXT Sensor I2C to Arduino:
 *  Arduino analog input 5 - I2C SCL
 *  Arduino analog input 4 - I2C SDA
 *  Both signals require a pull-up resistor (82KOhm) to Vdd.
 *
 *  Notes:
 *  The NXT is the Master and we are the slave IIC device.
 *  Lego NXT IIC operates at approximately 9600Baud, but as we are  
 *  the slave we don't have to do anything specific for this.
 *  RobotC can support faster IIC operation and this device should
 *  be able to work with that too.
 *
 *  All messages start with our slave address which is defined
 *  by ARDUPILOT_I2C_ADDRESS (the twi.c code takes care
 *  of only responding to messages which start with this
 *  address). Most Lego Sensors use an address of 0x01 as 
 *  there is only one sensor connected per NXT IIC Sensor 
 *  interface port. (The address is a 7 bit field, the LSBit
 *  of the addressing byte in IIC is used to indicate read
 *  or write operations, thus for a read the byte is 0x0?
 *  and for write it is 0x0?) 
 *
 *  The NXT reads and writes data from/to memory 
 *  mapped registers, using an 8 bit register address.
 *  Multiple bytes can be read or written at a time, with
 *  sutiable NXT code, as the register address is automatically
 *  incremented after each byte has been read/written.
 *
 *  This code is generic and can be used to share any data
 *  with the NXT.  As the IIC actions required to behave as a
 *  slave device are triggered by interrupts (handled in twi.c)
 *  this code is very efficient - it wastes no time in loops.
 *
 *  Constant strings for the Device, Manufacturer and version
 *  which can be read by the NXT can be customised.
 *
 ************************************************************/

/*************************************************************
 *  Todo:
 *  1) Modify wire.c or twi.c to include a timeout so that we 
 *  can get out of a state where we are left driving SDA low,
 *  probably as a result of having missed a clock pulse.
 *  
 *  2) Do we need any #pragmas to ensure that the m_NXTInterfaceData
 *  structure is byte packed?
 *
 ************************************************************/

// Make these includes in the main sketch file
//#include <avr/io.h>
//#include <Wire.h>										// We need to call functions from the wire library


//---------------------------------------------------------------------
// Constant Definitions
//---------------------------------------------------------------------
#define ARDUNXT_I2C_ADDRESS			((uint8_t)(0x02))        	// Our IIC slave address (this is a 7 bit value)
//#define NXT_SOFTWARE_VERSION			(0x01)				// Software version byte which can be read by NXT
#define NXT_SHARED_CONST_DATA_OFFSET            (0x00)				// The address offset of the first read only register
#define NXT_SHARED_CONST_DATA_SIZE		(0x18)				// The number of bytes allocated to the const memory
#define NXT_SHARED_DATA_OFFSET			(0x40)				// The address offset of the first read/write register
#define NXT_SHARED_DATA_SIZE			(0x20)				// The number of bytes allocated to the shared memory (See below)
#define NXT_TRANSACTION_TIMEOUT			(100U)				// number of milliseconds since last valid data transfer before we timeout connection
#define NXT_VERSION				'V','1','.','0','0','/0','/0','/0'			// Version string - must be 8 characters
#define NXT_MANUFACTURER			'D','I','Y','D','r','o','n','e'				// Manufacturer string - must be 8 characters
#define NXT_DEVICE_NAME				'R','C','i','/','f','/0','/0','/0'			// Device Name string - must be 8 characters

// If you want an LED to indicate the state of the NXT connection define which pin it is on here
// obviously you will need to be careful not to make use of this pin elsewhere in your code.
// If you don't want an NXT status LED then do not define NXT_LED_PIN
#define NXT_LED_PIN		(13)


//---------------------------------------------------------------------
// Macro Definitions
//---------------------------------------------------------------------
#if defined(NXT_LED_PIN)
#define NXT_LED(state)	digitalWrite(NXT_LED_PIN, state)
#else
#define NXT_LED(state)	{}
#endif


//---------------------------------------------------------------------
// Type and Structure Definitions
//---------------------------------------------------------------------

const static union 
{
	// The NXT accesses data by register address, which
	// is used as the offset into this byte array
	byte		au8Raw[NXT_SHARED_DATA_SIZE];			// The size of this array must be at least as large as the size of the Fields structure below

	// The Arduino code accesses data by variable names
	// BE CAREFUL If you add or remove fields, or change their width (number of bytes)
	// as this will cause the register addresses to move - code in the NXT will need
	// to be modified to keep in sync.
	struct 
	{									// Register address (starting at NXT_SHARED_CONST_DATA_OFFSET)
		char	szVersion[8];
		char	szManufacturer[8];
		char	szDeviceName[8];
	} Fields;
} m_NXTInterfaceConstData = {
	NXT_VERSION,								// Version string - must be 8 characters
	NXT_MANUFACTURER,							// Manufacturer string - must be 8 characters
	NXT_DEVICE_NAME								// Device Name string - must be 8 characters
};

// This is the main structure used to enable the NXT to access variables
// which are meaningful on the Arduino.
// The NXT needs to be aware that the Arduino is little endian
static union 
{
	// The NXT accesses data by register address, which
	// is used as the offset into this byte array
	byte		au8Raw[NXT_SHARED_DATA_SIZE];			// The size of this array must be at least as large as the size of the Fields structure below

	// The Arduino code accesses data by variable names
	// BE CAREFUL If you add or remove fields, or change their width (number of bytes)
	// as this will cause the register addresses to move - code in the NXT will need
	// to be modified to keep in sync.
	struct 
	{									// Register address (starting at NXT_SHARED_DATA_OFFSET)
		byte	u8SoftwareVersion;		// 0x40
                byte    u8Command;                      // Simple Commands from NXT
		byte	u8MuxMode;			// 0x41 MUX/Mode 0,1 or 2
		byte	u8RawCh1;			// 0x42	Remote Control Input Channel 1
		byte	u8RawCh2;        		// 0x43 Remote Control Input Channel 2
                byte    u8RawCh3;                       // 0x44 n/a
                byte    u8RawCh4;                       // 0x45 n/a
                INT_8   i8Ch1;                          // Signed Remote Control Input Channel 1
                INT_8   i8Ch2;                          // Signed Remote Control Input Channel 2
                INT_8   i8Ch3;                          // Signed Remote Control Input Channel 3
                INT_8   i8Ch4;                          // Signed Remote Control Input Channel 4
		byte	u8Servo0;			// 0x52 PWM Servo Output
		byte	u8Servo1;			// 0x53 PWM Servo Output
		byte	u8Servo2;			// 0x54 PWM Servo Output
                byte    u8Servo3;
                GPSRecord  GPS;
	} Fields;
} m_NXTInterfaceData;


//---------------------------------------------------------------------
// Variable declarations
//---------------------------------------------------------------------
static byte			m_u8NXTNumReceived;			// Count of number of bytes received (diagnostics)
static byte			m_u8NXTNumRequests;			// Count of number of bytes requested (diagnostics)
static byte			m_u8NXTAddress;				// Register address that the NXT is currently accessing
static unsigned long    	m_u32NXTLastRequest;            	// Time (in millis) at which the last valid request occured
static bool			m_bNXTAlive;				// Record of whether we believe that NXT communications are alive

// A few sanity checks
#if (255 < (NXT_SHARED_DATA_OFFSET + NXT_SHARED_DATA_SIZE))
#error 'NXT Code only only supports single byte for address'
#endif

//---------------------------------------------------------------------
// Global Functions
//---------------------------------------------------------------------

// Initialisation - call once when starting up 
void Init_NXTIIC(void)
{ 
	int	 i;

	// Initialise the Wire Library (this is the I2C (TWI/SMBus) library
	Serial.println("Init_NXTIIC");

	// Initialise NXT status LED output pin
#if defined(NXT_LED_PIN)
	pinMode(NXT_LED_PIN, OUTPUT);				// LED pin configured as an output
#endif

	Wire.begin(ARDUNXT_I2C_ADDRESS);			// Initialise "wire" and tell it what slave address we are using
	Wire.onRequest(NXTOnRequest);				// Register function to be called when NXT requests data 
	Wire.onReceive(NXTOnReceive);				// Register function to be called we receive data from the NXT 

	// Initialise variables
	m_u8NXTNumReceived = 0U;
	m_u8NXTNumRequests = 0U;
	m_u8NXTAddress = 0U;
	m_u32NXTLastRequest = 0U;
	m_bNXTAlive = false;

	// Initialise NXT shared data to 0
	for (i = 0; i < NXT_SHARED_DATA_SIZE; i++)
	{
		m_NXTInterfaceData.au8Raw[i] = 0;
	}
}  


// Callback function for when NXT requests a byte from us
void NXTOnRequest(void)
{
	if (!m_bNXTAlive)
	{
		// Connection not yet in use - we should receive an address before any read requests
		Wire.send(0);							// Dummy error return to avoid causing IIC to stall
		return;
	}
	// Send one or more bytes...
	if (m_u8NXTAddress < NXT_SHARED_DATA_OFFSET)
	{
		// Requested data is from the constant read only section
		byte	u8Offset;

		// Calculate the offset into the shared memory array
		u8Offset = m_u8NXTAddress - NXT_SHARED_CONST_DATA_OFFSET;

		if (u8Offset < NXT_SHARED_CONST_DATA_SIZE)
		{
			Wire.send(m_NXTInterfaceConstData.au8Raw[u8Offset]);
			
			// Auto increment to next byte - so that NXT can make multi-byte requests efficiently
			m_u8NXTAddress++;
		}
		else
		{
			// Out of range register address requested
		}
		// Auto increment to next byte - so that NXT can make multi-byte requests efficiently
		m_u8NXTAddress++;
	}
	else
	{
		// Data requested from shared memory area
		byte	u8Offset;

		// Calculate the offset into the shared memory array
		u8Offset = m_u8NXTAddress - NXT_SHARED_DATA_OFFSET;

		if (u8Offset < NXT_SHARED_DATA_SIZE)
		{
			Wire.send(m_NXTInterfaceData.au8Raw[u8Offset]);
			
			// Auto increment to next byte - so that NXT can make multi-byte requests efficiently
			m_u8NXTAddress++;
		}
		else
		{
			// Out of range register address requested
		}
	}
	m_u32NXTLastRequest = millis();
	m_u8NXTNumRequests++;						// Increment count of the number of valid bytes requested from us
}


// Callback function for when we receive one or more bytes from NXT
// All bytes received up to the IIC "stop" signal are received here in one go
// hence we do not need bData to be static retained across multiple calls.
void NXTOnReceive(int NumBytesReceived)
{
	bool	bData = false;

	if (!m_bNXTAlive)
	{
		// Connection not yet in use - it is now
                // don't call digitalWrite to turn LED on here as it is verbose and we are in an interrupt routine
		m_bNXTAlive = true;						// Remember that it is now working
	}

	while (NumBytesReceived--)
	{
		if (!bData)
		{
			// First byte we receive is the register address
			m_u8NXTAddress = Wire.receive();
			bData = true;						// Having received the address any further data is being written to us 
		}
		else 
		{
			// Subsequent bytes we receive are data
			if (m_u8NXTAddress < NXT_SHARED_DATA_OFFSET)
			{
				// NXT is attempting to write to an address which is below the shared memory area
			}
			else
			{
				byte	u8Offset;	

				// Calculate the offset into the shared memory array
				u8Offset = m_u8NXTAddress - NXT_SHARED_DATA_OFFSET;
			
				// Check that offset is in range
				if (u8Offset < NXT_SHARED_DATA_SIZE)
				{
					m_NXTInterfaceData.au8Raw[u8Offset] = Wire.receive();
					m_u8NXTNumReceived++;			// Increment count of the number of valid data bytes we have received
					m_u8NXTAddress++;				// Auto increment register address to support multi-byte transfers
				}
				else
				{
					// Request is out of range
					// Do not increment register address in this case to avoid it wrapping back round
					// and appearing to be back in a valid range
				}
				m_u32NXTLastRequest = millis();
			}
		}
	}
}


// Handler to synchronise data between NXT shared memory and other parts of the system
void NXTHandler(void)
{
	if (m_bNXTAlive)
	{
 		NXT_LED(HIGH);				// Switch NXT status LED On
	
		// Check if the connection is still alive
		if ((millis() - m_u32NXTLastRequest) > NXT_TRANSACTION_TIMEOUT)
		{
			// Timeout
			NXT_LED(LOW);			// Switch NXT status LED Off
			m_bNXTAlive = false;
			m_u8NXTAddress = 0;		// Reset register address
		}

		if (m_u8NXTNumReceived)
		{
			// We have received some data from NXT - so something has been changed
                        // Decode and handle COMMANDS from NXT
                        switch (m_NXTInterfaceData.Fields.u8Command)
                        {
                        case 1:      // Set RCInput Centre values to current stick positions
                          RCInput_SetCentre();
                          break;
                          
                        default:
                           break;
                        }
                        m_NXTInterfaceData.Fields.u8Command = 0;  // Clear any command now that we have done it
                        
                        // While the number of variables is small it is easier to just update everything,
			// if there were a lot more we might want to track which specific fields had been changed.
			//

			// Values in Fields can be used directly by other code, or we can take notice of them regularly here.
			// Examples
			// ========
			pulse_servo_0(m_NXTInterfaceData.Fields.u8Servo0);
			pulse_servo_1(m_NXTInterfaceData.Fields.u8Servo1);
			pulse_servo_2(m_NXTInterfaceData.Fields.u8Servo2);
			pulse_servo_3(m_NXTInterfaceData.Fields.u8Servo3);
			
                        m_u8NXTNumReceived = 0;
		}

		// Values in Fields (to be read by tbhe NXT) could be written to directly by applicable code throught the system,
		// or we can update them regularly here.
		NXTUpdateValues();
	}

//	NXTDiagnostics();	// If you want to enable this remove teh m_u8NXTNumReceived = 0; line above
}


// Function to update fields in the NXT shared memory area
static void NXTUpdateValues(void)
{
  // Examples
  // ========
  m_NXTInterfaceData.Fields.u8MuxMode = DigitalInput_Switch();
  
  if (TRUE)
  {
    // RCInput values (may) have been updated
    m_NXTInterfaceData.Fields.i8Ch1     = RCInput_Ch(0) / 4;	// 8 bit version of Signed PWM input
    m_NXTInterfaceData.Fields.u8RawCh1  = RCInput_Ch(0) >> 3;   // 8 bit version of Raw (unsigned) PWM input
    m_NXTInterfaceData.Fields.i8Ch2     = RCInput_Ch(1) / 4;	// 8 bit version of Signed PWM input
    m_NXTInterfaceData.Fields.u8RawCh2  = RCInput_Ch(1) >> 3;   // 8 bit version of Raw (unsigned) pWM input
    m_NXTInterfaceData.Fields.i8Ch3     = RCInput_Ch(2) / 4;	// 8 bit version of Signed PWM input
    m_NXTInterfaceData.Fields.u8RawCh3  = RCInput_Ch(2) >> 3;   // 8 bit version of Raw (unsigned) PWM input
    m_NXTInterfaceData.Fields.i8Ch4     = RCInput_Ch(3) / 4;	// 8 bit version of Signed PWM input
    m_NXTInterfaceData.Fields.u8RawCh4  = RCInput_Ch(3) >> 3;   // 8 bit version of Raw (unsigned) PWM input
  }
  
  if (g_GPSMsgFlags.bUpdate)
  {
    g_GPSMsgFlags.bUpdate = FALSE;
    m_NXTInterfaceData.Fields.GPS = m_GPSNew;
  }
}


static void NXTDiagnostics(void)
{
	if (m_u8NXTNumReceived)
	{
		Serial.print("Received ");
	    Serial.println((int)m_u8NXTNumReceived);
		m_u8NXTNumReceived = 0;
	}
	if (m_u8NXTNumRequests)
	{
		Serial.print("Requested ");
	    Serial.println((int)m_u8NXTNumRequests);
		m_u8NXTNumRequests = 0;
	}
}























/*
void pressure_i2c(void)
{
  int data = 0;
  
  // Receive data
  Wire.beginTransmission(SCP1000_ADDRESS);
  Wire.send(SNS_ADDR_PSTATUS);
  Wire.endTransmission();
  
  Wire.beginTransmission(SCP1000_ADDRESS);
  Wire.requestFrom(SCP1000_ADDRESS, (uint8_t)1);
  if (Wire.available())
  {
    data = Wire.receive();
    // Status Flags
    if (0U != (0x01 & data))
    {
    	//Startup procedure is still running
        Serial.print("Starting ");
        Serial.println(data); 
    }
    else
    {
      if (0U != (0x10 & data))
      {
  	// Real Time Error - data was not read in time - clear by reading
        Serial.print("RTErr ");      
      }
      if (0U != (0x20 & data))
      {
         Serial.print("DRDY ");

   	// DRDY - new data ready
   
        // Read Temperature 
        Wire.endTransmission();
        Wire.beginTransmission(SCP1000_ADDRESS);
        Wire.send(SNS_ADDR_PTEMP);
        Wire.endTransmission();
  
        Wire.beginTransmission(SCP1000_ADDRESS);
        Wire.requestFrom(SCP1000_ADDRESS, (uint8_t)2);
        
        // result is 16 bit - one byte at a time  
        if (Wire.available())
        {
          data = Wire.receive();
        }
        if (Wire.available()) 
        {
          data |= Wire.receive() << 8;
          Serial.print(data);
          Serial.print(",");
        }
        
        // Read Pressure
//      Wire.endTransmission();
        Wire.beginTransmission(SCP1000_ADDRESS);
        Wire.send(SNS_ADDR_PSTATUS);
//      Wire.endTransmission();
  
        Wire.beginTransmission(SCP1000_ADDRESS);
        Wire.requestFrom(SCP1000_ADDRESS, (uint8_t)2);
        
        // result is 16 bit - one byte at a time  
        if (Wire.available())
        {
          data = Wire.receive();
        }
        if (Wire.available()) 
        {
          data |= Wire.receive() << 8;
          Serial.print(data);
        }
      }
      Serial.println("");
    }
  }
  else
  {
	  // Nothing returned
	  Serial.print("X");
  }
  //Wire.endTransmission();
  // Use data
}



// prbably best to recode as a statemachine...
void thermo_i2c(void)
{
  int data = 0;
  int PEC = 0;
  
  // Receive data
  Wire.beginTransmission(THERMO_ADDR);
  Wire.send(SNS_ADDR_THERMO_TA);
  Wire.endTransmission();
  
  Wire.beginTransmission(THERMO_ADDR);
  Wire.requestFrom(THERMO_ADDR, (uint8_t)3);
  if (Wire.available())
  {
    data = Wire.receive();
       
    // result is 16 bit - one byte at a time  
    if (Wire.available())
    {
      data |= Wire.receive() << 8;
    }
    Serial.print(data);
    Serial.print(",");
	
	if (Wire.available()) 
    {
      PEC = Wire.receive();
    } 
	Serial.print(PEC);
    Serial.println("");
  }
  else
  {
	  // Nothing returned
	  Serial.print("X");
  }
  //Wire.endTransmission();
  // Use data
}


*/
