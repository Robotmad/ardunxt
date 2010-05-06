/* ArduNXT - Arduino based Lego Mondstorms NXT universal Remote Control Interface */
/* Please see http://code.google.com/p/ardunxt/ for further details and the latest version */

/*************************************************************
 *  DSM2 Receiver Serial Interface
 *  Written by Christopher Barnes
 *  April 2010 ...
 *
 *  Connect DSM2 Satellite SPM9545 Receiver to Arduino Serial RX
 *
 *  Notes:
 *  You need to bind the satellite reciever with the transmitter 
 *  by connecting it to a satellite compatible recevier (e.g. AR6200)
 *  before it can be used with this code.  It may be possible to provide
 *  support for binding in future.
 *
 */

/*
 Protocol:
  115200 Baud
  Word = HIGH Byte LOW Byte
  W1 W2 W3 W4 W5 W6 W7 W8

  W1 status:
  MSByte (First):
  D01-00 = Quality (higher is better)
  D04	 = Frame Loss 
  D05    = Repeated Frame Loss
  LSByte = 0x01	

  W2-8 are channels:
  D09-00 = Value (centre = 512)
  D13-10 = Channel Number

 Issues:
  How to detect/sync with frames (7mS inter frame gap or pattern matching)
*/

//#ifdef _DSM2_RX

#define MAX_NUM_CHANNELS	(7)
#define DSM2_SERIAL_BAUD	(115200)		   // Baud Rate used for DSM2 satellite receiver	


// Status LED:
// Off -      No Serial Input detected
// Flashing - Data present but no valid frames
// On -       Data present and valid frames received recently

// If you want an LED to indicate the state of the Receiver define which pin it is on here
// obviously you will need to be careful not to make use of this pin elsewhere in your code.
// If you don't want a Receiver status LED then do not define DSM2_LED_PIN
#define DSM2_LED_PIN			           		(12)

// Define the maximum allowed time between receiving valid frames for the LED to remain on
#define DSM2_LED_TIMEOUT                        (40)	 // milli seconds
#define DSM2_LED_FLASH_PERIOD                   (500)   // milli seconds flash period when searching 

//---------------------------------------------------------------------
// Macro Definitions
//---------------------------------------------------------------------
#if defined(DSM2_LED_PIN)
#define DSM2_LED(state)	digitalWrite(DSM2_LED_PIN, state)
#define DSM2_LED_STATE() digitalRead(DSM2_LED_PIN)
#else
#define DSM2_LED(state)	{}
#define DSM2_LED_STATE() (TRUE)
#endif

#define DSM2BUFF_SIZE				(14U)	// 2 bytes per channel

// DSM2 reception state machine states
#define STATE_WAITING_FOR_START		1
#define STATE_WAITING_FOR_LSB		2
#define STATE_COUNTING_BYTES 		3


// DSM2 protocol decoding definitions

// Flags to indicate which aspects of the GPS data have been received
typedef union {
  struct {
    unsigned bValid:1;          // DSM2 Status
    unsigned bUpdate:1;         // Flag to indicate that data has been updated (i.e. a full set of new values)
    unsigned bPresent:1;        // DSM2 module detected
  };
  struct {
    UINT_8 u8Value;
  };
} DSM2MsgFlags;

// Global variables
unsigned int					 g_u16DSM2Ch[MAX_NUM_CHANNELS];
DSM2MsgFlags	        		 g_DSM2MsgFlags;

// Local variables
static byte  m_u8RxByte;    
static byte *m_pDSM2;
static byte  m_u8DSM2Quality;	
static byte  m_u8RxState;
static byte  g_au8DSM2Buffer0[DSM2BUFF_SIZE];
static byte  g_u8DSM2WriteIndex0;
static byte  m_u8FrameBytes;
static unsigned long DSM2_timer;


void Init_DSM2(void)
{
  Serial.println("Init_DSM2");  
  m_u8RxState = STATE_WAITING_FOR_START;
  m_pDSM2 = g_au8DSM2Buffer0;
  g_u8DSM2WriteIndex0 = 0U;
  DSM2_timer = 0U;
  g_DSM2MsgFlags.u8Value = 0U;
  m_u8DSM2Quality = 0U;

  // Initialise DSM2 status LED output pin
#if defined(DSM2_LED_PIN)
  pinMode(DSM2_LED_PIN, OUTPUT);				// LED pin configured as an output
#endif

  // (re)Initialise Serial Port
#if (SERIAL_BAUD != 115200)
  Serial.begin(DSM2_SERIAL_BAUD); 
#endif
}

/****************************************************************
*
****************************************************************/
void DSM2_Handler(void)
{
  bool bExit = FALSE;
  
  // Process any characters which have been received.
  while (!bExit && (Serial.available() > 0))
  {
    m_u8RxByte = Serial.read();    
    
	// Serial.write(m_u8RxByte);

    switch (m_u8RxState)
    {
    case STATE_WAITING_FOR_START:	
      if ((m_u8RxByte & 0xCC) == 0x00)		// First byte of frame
      {
        // No illegal bits set 
        m_u8DSM2Quality = m_u8RxByte & 0x03;
		m_u8RxState = STATE_WAITING_FOR_LSB;
      }
      break;

    case STATE_WAITING_FOR_LSB:
      if (m_u8RxByte == 0x01)			// Second byte of frame
      {
        // Don't bother to save the byte as we know it must have been 0x01
	    g_u8DSM2WriteIndex0 = 0U;
        m_u8FrameBytes = DSM2BUFF_SIZE;	// If different transmitters use different length frames we could handle that here				
		// Serial.print("F:");  
	    m_u8RxState = STATE_COUNTING_BYTES;
      }
	  else
	  {
		// Illegal value - so it can't be the start of the frame
        m_u8RxState = STATE_WAITING_FOR_START;
	  }
      break;

    case STATE_COUNTING_BYTES:
      // Save frame payload data
      DSM2_Byte();
      if (--m_u8FrameBytes == 0U)
      {
        //Have just counted all data in
        // Valid message received complete
        m_pDSM2 = g_au8DSM2Buffer0;
        g_DSM2MsgFlags.bPresent = TRUE;
        DSM2_Decode();
        bExit = TRUE;  // Don't allow more than one message to be decoded per execution of the handler
	    m_u8RxState = STATE_WAITING_FOR_START;
      }
      break;	

    default:
      m_u8RxState = STATE_WAITING_FOR_START;
      break;        
    }
  }
  
  // Check for timeout
  if (g_DSM2MsgFlags.bValid) // Not really used at present - but would be necessary if a future receiver requires multiple frames for a full update
  {
    if(millis() - DSM2_timer > DSM2_LED_TIMEOUT)
    {
      DSM2_LED(LOW);				// If we don't receive any valid fixses in defined time turn off gps fix LED...
      DSM2_timer = millis();                // restart timer for next period
    
      // Discard any partially received data  
      g_DSM2MsgFlags.bValid = FALSE;		
    }
  }
  else if (g_DSM2MsgFlags.bPresent)
  {
    // Although we don't have a frame we have detected the presence of data - so we are searching for frames
    if (DSM2_timer)
    {
      if ((millis() - DSM2_timer) > DSM2_LED_FLASH_PERIOD)
      {
        // On or Off period has finished...
        DSM2_LED(DSM2_LED_STATE()?LOW:HIGH);   // Toggle LED On/Off 
        DSM2_timer = millis();                 // restart timer for next period
        g_DSM2MsgFlags.bPresent = FALSE;       // We need to receive something from the DSM2 receiver to indicate that it is present
      }
    }
    else
    {
        DSM2_LED(HIGH);                       // Turn LED On 
        DSM2_timer = millis();                // Start timer for On period
    }
  }
  else if (DSM2_timer)
  {
    if ((millis() - DSM2_timer) > DSM2_LED_FLASH_PERIOD)
    {
      // DSM2 satellite receiver not present
      DSM2_LED(LOW);
      DSM2_timer = 0U;
    }
  }
}

// Decode frame pointed to by m_pDSM2;
bool DSM2_Decode(void)
{
	byte u8Byte;
	byte u8Channel;
	byte i;

	// Serial.print("D");  
	
	for (i = 0; i < MAX_NUM_CHANNELS; i++)
	{
		u8Byte = *m_pDSM2++;
	    u8Channel = u8Byte >> 2;
		// Serial.print((int)u8Channel);
		if (u8Channel < MAX_NUM_CHANNELS)
	    {
			// Valid channel number
			g_u16DSM2Ch[u8Channel] = ((u8Byte & 0x03) << 8) + *m_pDSM2++;
		}
		else
		{
			// Channel number error
			// Serial.print("E");  
			return (FALSE);
		}
	}
    g_DSM2MsgFlags.bValid = TRUE;

	// Do we have a complete set of good data (yes because we only require a single frame at present)
	if (g_DSM2MsgFlags.bValid)
	{
		// We have a full new set of data - use it as an update
		g_DSM2MsgFlags.bUpdate = TRUE;

		// Convert received values to pulse widths
        for (i = 0U; i < NUM_RCI_CH; i++)
	    {
			g_u16Pulse[i] = 3 * g_u16DSM2Ch[i];		// (160 - 500 - 850 ish values received)
            g_RCIFlags[i].bValid = TRUE;
            g_RCIFlags[i].bUpdate = TRUE;           
		}
		// Mark all components as false so that we do not recognise another update until all have been refreshed
		g_DSM2MsgFlags.bValid = FALSE;
    
		if (g_DiagnosticsFlags.bRCInput) DSM2_Diagnostics();
		DSM2_timer = millis();    // Remember time of frame (for timeout)   
		DSM2_LED(HIGH);           // Turn LED On when valid frame is received.  
	}
	return(TRUE);
}


// DSM2 frame data buffering
// Protection against buffer overrun is provided by the check on the message length 
void DSM2_Byte(void)
{
  g_au8DSM2Buffer0[g_u8DSM2WriteIndex0++] = m_u8RxByte;
}


void DSM2_Diagnostics(void)
{
  // Diagnostics for DSM2
  Serial.print("DSM2: ");
  Serial.print((int)m_u8DSM2Quality);
  Serial.print(", ");
  Serial.print((int)g_u16DSM2Ch[0]);
  Serial.print(", ");
  Serial.print((int)g_u16DSM2Ch[1]);
  Serial.print(", ");
  Serial.print((int)g_u16DSM2Ch[2]);
  Serial.print(", ");
  Serial.print((int)g_u16DSM2Ch[3]);
  Serial.print(", ");
  Serial.print((int)g_u16DSM2Ch[4]);
  Serial.print(", ");
  Serial.print((int)g_u16DSM2Ch[5]);
  Serial.print(", ");
  Serial.print((int)g_u16DSM2Ch[6]);
  Serial.println(".");
}

//#endif	// _DSM2_RX
	

