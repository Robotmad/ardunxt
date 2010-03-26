/****************************************************************
* Here you have all the parsing stuff for uBlox
****************************************************************/

// Status LED:
// Off -      No GPS detected
// Flashing - GPS present but no valid position
// On -       GPS present and valid position received recently

// If you want an LED to indicate the state of the GPS define which pin it is on here
// obviously you will need to be careful not to make use of this pin elsewhere in your code.
// If you don't want an GPS status LED then do not define GPS_LED_PIN
#define GPS_LED_PIN              		(12)

// Define the maximum allowed time between receiving valid fixes for the LED to remain on
#define GPS_LED_TIMEOUT                         (1100)  // milli seconds (appropriate for 1Hz GPS)
#define GPS_LED_FLASH_PERIOD                    (500)   // milli seconds flash period when searching 

//---------------------------------------------------------------------
// Macro Definitions
//---------------------------------------------------------------------
#if defined(GPS_LED_PIN)
#define GPS_LED(state)	digitalWrite(GPS_LED_PIN, state)
#define GPS_LED_STATE() digitalRead(GPS_LED_PIN)
#else
#define GPS_LED(state)	{}
#define GPS_LED_STATE() (TRUE)
#endif



//You have to disable all the other string, only leave this ones:
//NAV - POSLLH Geodetic Position Solution, PAGE 66 of datasheet
//NAV - VELNED Velocity Solution in NED, PAGE 71 of datasheet
//NAV - STATUS Receiver Navigation Status, PAGE 67 of datasheet

#define GPSBUFF_SIZE				(80U)

// UBX reception state machine states
#define STATE_WAITING_FOR_UBX1		1
#define STATE_WAITING_FOR_UBX2		2
#define STATE_WAITING_FOR_UBXCLASS	3
#define STATE_WAITING_FOR_UBXID		4
#define STATE_WAITING_FOR_UBXLO		5
#define STATE_WAITING_FOR_UBXHI		6
#define STATE_COUNTING_UBX		7
#define STATE_WAITING_FOR_SUM_A		8
#define STATE_WAITING_FOR_SUM_B		9

// UBX protocol decoding definitions
#define UBX_MSG_ID_INDX				(0U)
#define UBX_MSG_LEN_INDX			(1U)
#define UBX_MSG_DATA_INDX			(2U)
#define UBX_VELNED_MSG_LEN			(36U)
#define UBX_STATUS_MSG_LEN			(16U)
#define UBX_POSLLH_MSG_LEN			(28U)
#define UBX_VELNED_SOG_INDX			(UBX_MSG_DATA_INDX + 20U)
#define UBX_VELNED_COG_INDX			(UBX_MSG_DATA_INDX + 24U)
#define UBX_STATUS_FIX_INDX			(UBX_MSG_DATA_INDX + 4U)
#define UBX_POSLLH_TOW_INDX	        	(UBX_MSG_DATA_INDX + 0U)	// millisecond Time Of Week
#define UBX_POSLLH_LON_INDX			(UBX_MSG_DATA_INDX + 4U)
#define UBX_POSLLH_LAT_INDX			(UBX_MSG_DATA_INDX + 8U)
#define UBX_POSLLH_ALT_INDX			(UBX_MSG_DATA_INDX + 16U)	// Height above Mean Sea Level

// Local variables
byte  g_u8RxByte;    
byte *m_pGPS;
byte  g_u8RxState;
byte  g_au8GPSBuffer0[GPSBUFF_SIZE];
byte  g_u8GPSWriteIndex0;
byte  m_u8CountDownBytes;
byte  m_UBX_Checksum_B;
byte  m_UBX_Checksum_A; 
unsigned long GPS_timer;


void Init_GPS(void)
{
  Serial.println("Init_GPS");  
  g_u8RxState = STATE_WAITING_FOR_UBX1;
  m_pGPS = g_au8GPSBuffer0;
  g_u8GPSWriteIndex0 = 0U;
  GPS_timer = 0U;
  g_GPSMsgFlags.u8Value = 0U;

  // Initialise NXT status LED output pin
#if defined(GPS_LED_PIN)
  pinMode(GPS_LED_PIN, OUTPUT);				// LED pin configured as an output
#endif
}

/****************************************************************
*
****************************************************************/
void GPS_Handler(void)
{
  bool bExit = FALSE;
  
  // Process any characters which have been received.
  while (!bExit && (Serial.available() > 0))
  {
    g_u8RxByte = Serial.read();    
    
    switch (g_u8RxState)
    {
    case STATE_WAITING_FOR_UBX1:
      if (g_u8RxByte == 0xB5U)		// First byte of UBX packet
      {
        g_u8RxState = STATE_WAITING_FOR_UBX2;
      }
      GPSByte();                        // Not a UBX byte - try processing as NMEA
      break;
				
    case STATE_WAITING_FOR_UBX2:
      if (g_u8RxByte == 0x62U)		// Second byte of UBX packet
      {
        g_u8RxState = STATE_WAITING_FOR_UBXCLASS;
      }
      else
      {
        g_u8RxState = STATE_WAITING_FOR_UBX1;
      }
      GPSByte();
      break;

    case STATE_WAITING_FOR_UBXCLASS:
      if (g_u8RxByte == 0x01U)
      {
        // Only interested in UBX Class 1 messages
        if (UBX_StartMsg())
        {
          g_u8RxState = STATE_WAITING_FOR_UBXID;
        }
        else
        {
          // Can't receive message
          g_u8RxState = STATE_WAITING_FOR_UBX1;					
        }
      }
      else
      {
//      Serial.println("C");  
        g_u8RxState = STATE_WAITING_FOR_UBX1;
        GPSByte();
      }
      break;
							
    case STATE_WAITING_FOR_UBXID:
      // This is the first variable byte of the UBX messages of interest
      UBX_Byte();					
      UBX_Checksum();
      g_u8RxState = STATE_WAITING_FOR_UBXLO;
      break;
			
    case STATE_WAITING_FOR_UBXLO:	// Message length LSB
      if (g_u8RxByte < (GPSBUFF_SIZE - 1))	// Must be strictly less than as we have used up a byte from the buffer for the message ID & length
      {
        //Message is valid length
        UBX_Byte();
        UBX_Checksum();
        m_u8CountDownBytes = g_u8RxByte;				
        g_u8RxState = STATE_WAITING_FOR_UBXHI;
      }
      else
      {
        // UBX Message length too long for us
//      Serial.println("LL");  
        g_u8RxState = STATE_WAITING_FOR_UBX1;
        GPSByte();
      }
      break;

    case STATE_WAITING_FOR_UBXHI:	// Message length MSB
      if (g_u8RxByte == 0U)
      {
        //Message is valid length
        // Don't bother to save the byte as we know it must have been 0
        UBX_Checksum();
        g_u8RxState = STATE_COUNTING_UBX;
      }
      else
      {
        // UBX Message length too long for us
//      Serial.println("LH");  
        g_u8RxState = STATE_WAITING_FOR_UBX1;
        GPSByte();
      }
      break;

    case STATE_COUNTING_UBX:
      // Save message payload data
      UBX_Byte();
      UBX_Checksum();
      if (--m_u8CountDownBytes == 0U)
      {
        //Have just counted all data in
        g_u8RxState = STATE_WAITING_FOR_SUM_A;
      }
      break;	

    case STATE_WAITING_FOR_SUM_A:
      if (g_u8RxByte == m_UBX_Checksum_A)
      {
        g_u8RxState = STATE_WAITING_FOR_SUM_B;
      }	
      else
      {
        // UBX Message failed checksum
        Serial.println("X");  
        g_u8RxState = STATE_WAITING_FOR_UBX1;
        GPSByte();
      }
      break;

    case STATE_WAITING_FOR_SUM_B:
      if (g_u8RxByte == m_UBX_Checksum_B)
      {
        // Valid message received complete
        m_pGPS = g_au8GPSBuffer0;
        g_GPSMsgFlags.bPresent = TRUE;
        UBX_Decode();
        bExit = TRUE;  // Don't allow more than one message to be decoded per execution of the handler
      }	
      else
      {
        // UBX Message failed checksum
        Serial.println("X");  
        GPSByte();
      }
      // fall through

    default:
      g_u8RxState = STATE_WAITING_FOR_UBX1;
      break;        
    }
  }
  
  // Check for timeout
  if (g_GPSMsgFlags.bValid)
  {
    if(millis() - GPS_timer > GPS_LED_TIMEOUT)
    {
      GPS_LED(LOW);				// If we don't receive any valid fixses in defined time turn off gps fix LED...
      GPS_timer = millis();                // restart timer for next period
    
      // Discard any partially received data  
      g_GPSMsgFlags.bValid = FALSE;
      g_GPSMsgFlags.bTime = FALSE;
      g_GPSMsgFlags.bLatLon = FALSE;
      g_GPSMsgFlags.bAltitude = FALSE;
      g_GPSMsgFlags.bSpeed = FALSE;			
    }
  }
  else if (g_GPSMsgFlags.bPresent)
  {
    // ALthough we don't have a valid fix we have detected the presence of a GPS module - so we are searching for satellites
    if (GPS_timer)
    {
      if ((millis() - GPS_timer) > GPS_LED_FLASH_PERIOD)
      {
        // On or Off period has finished...
        GPS_LED(GPS_LED_STATE()?LOW:HIGH);   // Toggle LED On/Off 
        GPS_timer = millis();                // restart timer for next period
        g_GPSMsgFlags.bPresent = FALSE;      // We need to receive something from the GPS module to indicate that it is present
      }
    }
    else
    {
        GPS_LED(HIGH);                       // Turn LED On 
        GPS_timer = millis();                // Start timer for On period
    }
  }
  else if (GPS_timer)
  {
    if ((millis() - GPS_timer) > GPS_LED_FLASH_PERIOD)
    {
      // GPS Module not present
      GPS_LED(LOW);
      GPS_timer = 0U;
    }
  }
}

void UBX_Decode(void)
{
//Serial.print("D");  

  switch (m_pGPS[UBX_MSG_ID_INDX])
  {
  case 0x02: //ID NAV - POSLLH
    if (GPS_DecodePOSLLHMessage())
    {
      // NAV_POSLLH message decoded OK
      g_GPSMsgFlags.bTime = TRUE;
      g_GPSMsgFlags.bLatLon = TRUE;
      g_GPSMsgFlags.bAltitude = TRUE;
    }
    break;

  case 0x03:  //  ID NSV = STATUS
    if (GPS_DecodeSTATUSMessage())
    {
      // NAV_STATUS message decoded OK
      g_GPSMsgFlags.bValid = TRUE;
    }
    break;

  case 0x12:  // ID NAV - VELNED
    if (GPS_DecodeVELNEDMessage())
    {
      // NAV_VELNED message decoded OK
      g_GPSMsgFlags.bSpeed = TRUE;		// SOG and COG
    }
    break;	
    
  default:
    // Unexpected Message ID
    Serial.print("GPS-UBX NAV ");
    Serial.println((int)m_pGPS[UBX_MSG_ID_INDX]);
    break;  
  }
  
  // Do we have a complete set of good fix data
  if (g_GPSMsgFlags.bLatLon && g_GPSMsgFlags.bSpeed && g_GPSMsgFlags.bAltitude && g_GPSMsgFlags.bValid)
  {
    // We have a full new set of data - use it as an update
    g_GPSMsgFlags.bUpdate = TRUE;
    
    // Mark all components as false so that we do not recognise another update until all have been refreshed
    g_GPSMsgFlags.bLatLon  = FALSE;
    g_GPSMsgFlags.bSpeed  = FALSE;
    g_GPSMsgFlags.bAltitude  = FALSE;
    g_GPSMsgFlags.bValid = FALSE;
    
    if (g_DiagnosticsFlags.bGPS) GPS_Diagnostics();
    GPS_timer = millis();    // Remember time of fix (for timeout)   
    GPS_LED(HIGH);           // Turn LED On when valid GPS fix is received.  
  }
}


void UBX_Checksum(void)
{
  m_UBX_Checksum_A += g_u8RxByte;
  m_UBX_Checksum_B += m_UBX_Checksum_A; 
}

// UBX GPS buffering
// Protection against buffer overrun is provided by the check on the message length 
void UBX_Byte(void)
{
  g_au8GPSBuffer0[g_u8GPSWriteIndex0++] = g_u8RxByte;
}

bool UBX_StartMsg(void)
{
  m_UBX_Checksum_A = 0x01;	// First byte included in Checksum is Class = 0x01 
  m_UBX_Checksum_B = 0x01; 
  g_u8GPSWriteIndex0 = 0U;

  return(TRUE);  // can always use buffer as we do so forcibly
}


bool GPS_DecodeSTATUSMessage(void)
{
// 
// GPSfix Type 
// - 0x00 = no fix
// - 0x01 = dead reckoning
// - 0x02 = 2D-fix
// - 0x03 = 3D-fix
// - 0x04 = GPS + dead reckoning
// - 0x05 = Time only fix
// - 0x06..0xff = reserved

//Serial.print("GPS:STATUS ");  

  if (UBX_STATUS_MSG_LEN == (UINT_8)m_pGPS[UBX_MSG_LEN_INDX])
  {
    // Correct message length
    switch (m_pGPS[UBX_STATUS_FIX_INDX])
    {
    case 2:	// 2D-fix
    case 3:	// 3D-fix
    case 4:	// GPS + dead reckoning
//    Serial.println("OK");  
      return(TRUE);
      break;
      
    default:
//    Serial.print("GPS:STATUS ");  
//    Serial.println((int)m_pGPS[UBX_STATUS_FIX_INDX]);		
      break;	
    }
  }	
  return(FALSE);	
}

bool GPS_DecodeVELNEDMessage(void)
{
//Serial.print("GPS:VELNED ");  

  if (UBX_VELNED_MSG_LEN == (UINT_8)m_pGPS[UBX_MSG_LEN_INDX])
  {
    byte *m_pcField;
    INT_32	i32Temp;
		
    m_pcField = &m_pGPS[UBX_VELNED_SOG_INDX];
		
    // Ground Speed (2D)
    m_GPSNew.u16Speed = *((UINT_16 *)m_pcField);	// cm/s

    // Heading (100ths of a degree)
    m_pcField = &m_pGPS[UBX_VELNED_COG_INDX];
    i32Temp = *((INT_32 *)m_pcField);	// deg 1e-5
    i32Temp /= 1000;					// deg 1e-2
    if (0 > i32Temp)
    {
      i32Temp += 36000;
    }
    m_GPSNew.u16Heading = (UINT_16)i32Temp;

//  Serial.println("OK");  
    return(TRUE);
  }
  return(FALSE);
}

bool GPS_DecodePOSLLHMessage(void)
{
//Serial.print("GPS:POSLLH ");  

  if (UBX_POSLLH_MSG_LEN == m_pGPS[UBX_MSG_LEN_INDX])
  {
    byte       *m_pcField;
    INT_32	i32Temp;
		
    // Time
    m_pcField = &m_pGPS[UBX_POSLLH_TOW_INDX];
    i32Temp = *((UINT_32 *)m_pcField);	// mS (iTOW)
    m_GPSNew.u24Time = i32Temp / 1000;	// Seconds
    m_GPSNew.u8CentiSeconds = (i32Temp - ((UINT_32)m_GPSNew.u24Time * 1000)) / 10;

    // Longitude (10,000th Minutes)
    m_pcField = &m_pGPS[UBX_POSLLH_LON_INDX];
    i32Temp = *((INT_32 *)m_pcField);	// deg 1e-7
    i32Temp /= 10;						// deg 1e-6
    i32Temp *= 6;						// min 1e-5
    m_GPSNew.i32Longitude = i32Temp;

    // Latitude (10,000th Minutes)
    m_pcField = &m_pGPS[UBX_POSLLH_LAT_INDX];
    i32Temp = *((INT_32 *)m_pcField);	// deg 1e-7
    i32Temp /= 10;						// deg 1e-6
    i32Temp *= 6;						// min 1e-5
    m_GPSNew.i32Latitude = i32Temp;

    // Altitude (m/10)
    m_pcField = &m_pGPS[UBX_POSLLH_ALT_INDX];
    i32Temp = *((INT_32 *)m_pcField);	// mm
    i32Temp /= 100;						// m/10
    m_GPSNew.u16Altitude = (UINT_16)i32Temp;

//  Serial.println("OK");  
    return(TRUE);
  }
  return(FALSE);
}


// Generic NMEA GPS
void GPSByte(void)
{
  static bool  bStarted = FALSE;
  
  if (g_u8RxByte == '$')
  {
    // Start to buffer characters
    g_u8GPSWriteIndex0 = 0U;
    bStarted = TRUE;
  }
  else if (bStarted)
  { 
    if (g_u8RxByte == '\n') g_u8RxByte == '\r';	// Consistent line ending
    if (GPSBUFF_SIZE <= g_u8GPSWriteIndex0)
    {
      // abandon partial string
      bStarted = FALSE;
    }
    else
    {
      g_au8GPSBuffer0[g_u8GPSWriteIndex0++] = g_u8RxByte;
      if (g_u8RxByte == '\r')
      {
        // Attempt to decode
        NMEA_Decode();
      }
    }
  }
}

void NMEA_Decode(void)
{
  // TODO - import decoding from other projects  
  // g_GPSMsgFlags.bPresent = TRUE;
}

void GPS_Diagnostics(void)
{
  // Diagnostics for GPS    
  // Time
//  Serial.print("TOW: ");
  Serial.print(m_GPSNew.u24Time);
  Serial.print(".");
  Serial.print((int)m_GPSNew.u8CentiSeconds);
  Serial.print("S, ");
  // Longitude (10,000th Minutes)
//  Serial.print("Lon: ");
  Serial.print(m_GPSNew.i32Longitude);
//  Serial.print(" Min(/10,000)");
  Serial.print(", ");
  // Latitude (10,000th Minutes)
//  Serial.print("Lat: ");
  Serial.print(m_GPSNew.i32Latitude);
//  Serial.print(" Min(/10,000)");
  Serial.print(", ");
  // Altitude (m/10 scales to m)
//  Serial.print("Alt: ");
  Serial.print(m_GPSNew.u16Altitude/10);
//  Serial.print("m");
  Serial.print(", ");
  // Speed over ground (2D)
//  Serial.print("SOG: ");
  Serial.print(m_GPSNew.u16Speed);
//  Serial.print(" cm/s");
  Serial.print(", ");
  // Heading (100ths of a degree)
//  Serial.print("COG: ");
  Serial.print(m_GPSNew.u16Heading/100);
//  Serial.println(" Deg(/100)");  }
  Serial.println(" ");
}

//#endif
