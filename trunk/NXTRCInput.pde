// TODO 
// high level logic to check that we get a pulse for all expected channels
// to allow detection of which channels are active
// timeout (so that when pulses stop we clar bValid flags)
// failsafe

#define MAX_PULSE_WIDTH		(2200)					// uS Maximum pulse width considered as valid
#define MIN_PULSE_WIDTH		(800)					// uS Minimum pulse width considered as valid
#define DFLT_PWMI_CENTRE        (1500)                                  // uS Default center pulse width
#define PWM_PERIOD		(40000)                                 // PWM Output period (MUST be in sync with value used to set up T1)

#define NUM_PWMI	        (4)                                     // Number of RCInput (Pulse Width Measurement) Channels 

#if (NUM_RCI_CH < NUM_PWMI)
#error 'Error there must be at least as many RCI Channels as PWMI Channels'
#endif

// We have no choice of which pins are used for input channels 0 & 1 they must be PD2, PD3 (Arduino Digital inputs 2 and 3)
// We do however have some choice over the extra channels - but please be careful as they must be on different ports
#define RCINPUT_CH2_PIN         (6)                                     // RCInput pin on Port D
#define RCINPUT_CH3_PIN         (11)                                    // RCInput pin on Port B

// Flags for Remote Control PWM Input Channels
/* Moved to main file to make it available globally
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
    unsigned bWrap:1;        // Need to add PWM_PERIOD to fall time  
  };
  struct {
    UINT_8 u8Value;
  };
} 
RCInputFlags;
*/
#define HIGH_LEVEL_FLAG_MASK  (0x03)


/*************************************************************************
 * 
 *************************************************************************/
//volatile RCInputflags                                  g_RCIFlags[NUM_PWMI];
volatile unsigned int					 g_u16PRise[NUM_PWMI];
volatile unsigned int					 g_u16PFall[NUM_PWMI];
//unsigned int						 g_u16Pulse[NUM_PWMI];
unsigned int                                             g_u16Centre[NUM_RCI_CH]; 
unsigned int                                             m_u16ValidFrames; 


/**************************************************************
 * Configuring the RC Input channels  
 * This code depends on Timer1 running continuously as part of 
 * the Servo Output pulse width timing.
 ***************************************************************/
void Init_RCInputCh(void)
{   
  Serial.println("Init_RCInput");

  // First two RCInput channels (0 & 1) have hardware interrupt support (INT0 and INT1)
  pinMode(2, INPUT);      // RC Input pin 0
  pinMode(3, INPUT);      // RC Input pin 1

  digitalWrite(2, HIGH);   // Pull up so that channel is not left floating if there is nothing connected
  digitalWrite(3, HIGH);   // Pull up so that channel is not left floating if there is nothing connected

// Interrupts handled directly rather than via Arduino "attach" function as this introduces unnecessary extra code to the interrupt handler  
//  attachInterrupt(0, ISRRC0, CHANGE);	
//  attachInterrupt(1, ISRRC1, CHANGE);
  EICRA = _BV(ISC10) | _BV(ISC00);       // Interrupt on any change on either INT0 or INT1
  EIMSK = _BV(INT0) | _BV(INT1);         // Enable interrupts for INT0 and INT1 
  
  
  // Extra RCInput channels are managed by interrupt software
  // using pin change interrupts (one pin per port - so we don't have to detect which pin has changed)
  pinMode(RCINPUT_CH2_PIN, INPUT);       // RC Input pin 2
  pinMode(RCINPUT_CH3_PIN, INPUT);       // RC Input pin 3

  digitalWrite(RCINPUT_CH2_PIN, HIGH);   // Pull up so that channel is not left floating if there is nothing connected
  digitalWrite(RCINPUT_CH3_PIN, HIGH);   // Pull up so that channel is not left floating if there is nothing connected

  Init_RCInputPCIISR();

  for (int i=0; i < NUM_RCI_CH; i++)
  {  
    g_u16Centre[i] = DFLT_PWMI_CENTRE;    // TODO read from EEPROM
    g_RCIFlags[i].u8Value = 0U;         
  }

  m_u16ValidFrames = 0;
  // NB sei(); is required to enable interrupts
}


unsigned int RCInput_ValidFrames(void)
{
  if (60000 < m_u16ValidFrames)
  {
    // Prevent overflow
    m_u16ValidFrames = 60000;
  }
  return(m_u16ValidFrames);
}

/**************************************************************
 * Functions to read the most recent value from the RCInput channels 
 ***************************************************************/

unsigned int RCInput_RawCh(unsigned char u8Ch)
{
  return(g_u16Pulse[u8Ch]);
}

// Adjusting for the receiver stick centre position
int RCInput_Ch(unsigned char u8Ch)
{
  return((int)g_u16Pulse[u8Ch] - (int)g_u16Centre[u8Ch]);
}

// Function to set RCInput centre for all channels
void RCInput_SetCentre(void)
{
  unsigned char u8Ch;

  for (u8Ch = 0; u8Ch < NUM_RCI_CH; u8Ch++)
  {
    RCInput_SetCentre(u8Ch);
  }
}

// Function to set RCInput centre for a single channel
void RCInput_SetCentre(unsigned char u8Ch)
{
  g_u16Centre[u8Ch] = g_u16Pulse[u8Ch]; 
}





//
// Remote Control Pulse Width Measurement
// Any pulse could be first of set - i.e. no assumption about channel order
//
  
  
// Remote Control Inptut 0 (PD2)
//void ISRRC0(void)
ISR(INT0_vect)
{
  // Change of state on INT0

//if (digitalRead(2)==HIGH)        // This is very verbose code to include in an interrupt handler - hence optimised below
  if (bit_is_set(PIND, 2))
  {
    // Hard coded for active high pulse measurement

    // Rising edge
    if (g_RCIFlags[0].bRise)
    {
      // Oops we have not yet used the measurement made of the previous pulse
      g_RCIFlags[0].bLost = TRUE;
    }

    g_u16PRise[0] = TCNT1;					// Remember time of rising edge
    g_RCIFlags[0].bRise = TRUE;
    g_RCIFlags[0].bFall = FALSE;                                // Ignore any stale falling edge measurement
    g_RCIFlags[0].bTimerWrap = FALSE;                           // First edge of the pulse - so we can reset the Timer Wrap flag
  }
  else
  {
    // Falling edge
    if (g_RCIFlags[0].bRise)
    {
      // We already have a rising edge measurement - so we can use this falling edge
      g_u16PFall[0] = TCNT1;					// Remember time of falling edge
      g_RCIFlags[0].bFall = TRUE;
      g_RCIFlags[0].bWrap = g_RCIFlags[0].bTimerWrap;
    }
  }
}

// Remote Control Inptut 1
//void ISRRC1(void)
ISR(INT1_vect)
{
  // Change of state on INT1
//if (digitalRead(3)==HIGH)        // This is very verbose code to include in an interrupt handler - hence optimised below
  if (bit_is_set(PIND, 3))
  {
    // Hard coded for active high pulse measurement

    // Rising edge
    if (g_RCIFlags[1].bRise)
    {
      // Oops we have not yet used the measurement made of the previous pulse
      g_RCIFlags[1].bLost = TRUE;
    }

    g_u16PRise[1] = TCNT1;					// Remember time of rising edge
    g_RCIFlags[1].bRise = TRUE;
    g_RCIFlags[1].bFall = FALSE;                                // Ignore any stale falling edge measurement
    g_RCIFlags[1].bTimerWrap = FALSE;                           // First edge of the pulse - so we can reset the Timer Wrap flag
  }
  else
  {
    // Falling edge
    if (g_RCIFlags[1].bRise)
    {
      // We already have a rising edge measurement - so we can use this falling edge
      g_u16PFall[1] = TCNT1;					// Remember time of falling edge
      g_RCIFlags[1].bFall = TRUE;
      g_RCIFlags[1].bWrap = g_RCIFlags[1].bTimerWrap;
    }
  }
}



// Code to implement more RCInput channels using pin change interrupts
// By enabling only one input pin per 8 bit port when the interrupt on
// pin change is triggered we do not need to spend any time (in the interrupt routine)
// working out which pin has changed state - which keeps the interrupt routine
// small and fast - which is important as further interrupts are disabled while
// it is being handled.
//
// PCINT0  = PR0 = D8
// PCINT1  = PB1 = D9
// PCINT2  = PB2 = D10
// PCINT3  = PB3 = D11 (RCInput Ch4)
// PCINT4  = PB4 = D12
// PCINT5  = PB5 = D13
// 6, 7 used for Xtal
//
// 8 to 11 in use for analogue inputs
// 12, 13 used for IIC SDA & SCL
// 14 used for Reset input
// 15 does not exist
//
// 16, 17 in use for Serial Tx & Rx
// 18, 19 in use for INT0,1 external interrupts
// PCINT20 = PD4 = MUX
// PCINT21 = PD5 = D5
// PCINT22 = PD6 = D6 (RCInput Ch 3)
// PCINT23 = PD7 = D7

static void Init_RCInputPCIISR(void)
{
  // Enable pin change interrupts for specific pins
  // By enabling only one pin per group when it triggers we don't need to spend any time working out which pin has changed
  PCMSK0 = _BV(3);    // Only enable interrupt on change for "Pin 11" PB3 
  PCMSK2 = _BV(6);    // Only enable interrupt on change for "Pin 6" PD6

  PCICR = 0x05;       // Enable pin change interrupt for groups 0..7 and 16..23
}



// In the interrupt routines by default the compiler will NOT have re-enabled interrupts, so they will still be disabled
// PCINT0 is for PCINT0..7 (PB0..7)
#if ((8 > RCINPUT_CH3_PIN) || (15 < RCINPUT_CH3_PIN))
#error 'RCInput Channel 3 must be on Port B'
#endif
ISR(PCINT0_vect)
{
  // Change of state on a Port B pin
  // We are using arduino pin 11 on Port B
//if (digitalRead(RCINPUT_CH3_PIN)==HIGH)      // This is very verbose code to include in an interrupt handler - hence optimised below
  if (bit_is_set(PINB, RCINPUT_CH3_PIN - 8))
  {
    // Hard coded for active high pulse measurement

    // Rising edge
    if (g_RCIFlags[3].bRise)
    {
      // Oops we have not yet used the measurement made of the previous pulse
      g_RCIFlags[3].bLost = TRUE;
    }

    g_u16PRise[3] = TCNT1;					// Remember time of rising edge
    g_RCIFlags[3].bRise = TRUE;
    g_RCIFlags[3].bFall = FALSE;                                // Ignore any stale falling edge measurement
    g_RCIFlags[3].bTimerWrap = FALSE;                           // First edge of the pulse - so we can reset the Timer Wrap flag
  }
  else
  {
    // Falling edge
    if (g_RCIFlags[3].bRise)
    {
      // We already have a rising edge measurement - so we can use this falling edge
      g_u16PFall[3] = TCNT1;					// Remember time of falling edge
      g_RCIFlags[3].bFall = TRUE;
      g_RCIFlags[3].bWrap = g_RCIFlags[3].bTimerWrap;
    }
  }
}

// PCINT1 is for PCINT8..15 pins (PC0..7)
// No RCInput currently allocated to this port
ISR(PCINT1_vect)
{
  // Change of state on a Port C pin
}


// PCINT2 is for PCINT16..23 pins (PD0..7)
#if (7 < RCINPUT_CH2_PIN)
#error 'RCInput Channel 2 must be on Port D'
#endif
ISR(PCINT2_vect)
{
  // Change of state on a Port D pin
  // We are using arduino pin 6 on Port D
//if (digitalRead(RCINPUT_CH2_PIN)==HIGH)  // This is very verbose code to include in an interrupt handler - hence optimised below
  if (bit_is_set(PIND, RCINPUT_CH2_PIN))
  {
    // Hard coded for active high pulse measurement

    // Rising edge
    if (g_RCIFlags[2].bRise)
    {
      // Oops we have not yet used the measurement made of the previous pulse
      g_RCIFlags[2].bLost = TRUE;
    }

    g_u16PRise[2] = TCNT1;					// Remember time of rising edge
    g_RCIFlags[2].bRise = TRUE;
    g_RCIFlags[2].bFall = FALSE;                                // Ignore any stale falling edge measurement
    g_RCIFlags[2].bTimerWrap = FALSE;                           // First edge of the pulse - so we can reset the Timer Wrap flag
  }
  else
  {
    // Falling edge
    if (g_RCIFlags[2].bRise)
    {
      // We already have a rising edge measurement - so we can use this falling edge
      g_u16PFall[2] = TCNT1;					// Remember time of falling edge
      g_RCIFlags[2].bFall = TRUE;
      g_RCIFlags[2].bWrap = g_RCIFlags[2].bTimerWrap;
    }
  }
}





// TODO add timeout for no signal (configuration fow what to do in this case - return NO_VALUE, last know, center?
void RCInput_Handler(void)
{
  static  unsigned int u16Pulse;    // Static as that usually gives faster code
  static  unsigned int u16PFall;
  static  unsigned int u16PRise;
  byte	               i;

  if (g_DSM2MsgFlags.bPresent)
  {
	  // DSM2 satellite receiver in use for Remote Control
	  if (g_DSM2MsgFlags.bUpdate)
	  {
			m_u16ValidFrames++;
	  		g_DSM2MsgFlags.bUpdate = FALSE;
	  }
	  return;
  }

  // Loop over outstanding active channels 
  for (i = 0U; i < NUM_PWMI; i++)
  {
    // add code from 7003 project so that we only test channels we are still waiting for
    // Channel pending measurement

    if (g_RCIFlags[i].bFall)
    {
      // I want to make the code while interrupts are disabled as quick as possible - so trying to get the 
      // compiler to do some of the address manipulation here first.  
      volatile RCInputFlags  *pRCIFlags = &g_RCIFlags[i];

      //Serial.print((int)i);
      //Serial.print(":");

      // Both rise and fall edges have been timestamped
      // Take a copy of the rise and fall timestamps so that they can be updated in the interrupt routine for the next pulse
      u16PFall = g_u16PFall[i];
      u16PRise = g_u16PRise[i];

      if (pRCIFlags->bWrap)
      {
        // Timer1 wraped around during the pulse measurement - compensate result
        // This is normal behaviour as the timer we are using is not in sync with the received frames
        //Serial.print("+");
        u16PFall += PWM_PERIOD;  // Can't overflow 16 bits for valid pulse widths
      }
      if (pRCIFlags->bLost)
      {
        // We have received more than one pulse since we last computed the pulse width from the rise and fall times
        // The current (most recent) pulse record is still valid and can be used OK
        // if this happens a lot then the this handler is not being called frequently enough
        Serial.print((int)i);
        Serial.println(" ***LostPulse***");
//      m_u16LostPulses++;
        pRCIFlags->bLost = FALSE;
      }

      // It is just possible that while we copied the timestamps we started to measure a new pulse
      // in which case the values do not form a pair - we can tell if this happened because
      // when an edge is timestamped the flag for the opposite edge is cleared.  Thus if both flags are still set we 
      // are safe to use thse values.

      cli();  // Disable interrupts 
      if (!pRCIFlags->bFall)
      {
        // One of the flags has been cleared so a new pulse is being measured
        sei();  // Re-enable interrupts
        Serial.print((int)i);
        Serial.println(" ***MidPulse***");
      }
      else
      {
        // Clear all low level flags to start a new measurement (i.e. just leave the high level flags as set)
        pRCIFlags->u8Value &= HIGH_LEVEL_FLAG_MASK;

        sei();  // re-enable interrupts

        // A pair of rise and fall time measurements have been made
        //Serial.print((int)i);
        //Serial.print(":");
        //Serial.print(u16PRise);
        //Serial.print(",");
        //Serial.print(u16PFall);
        //Serial.print(" ");

        u16Pulse = (u16PFall - u16PRise);

        // Scale measurement down to uS
        u16Pulse >>= 1;

        if (MIN_PULSE_WIDTH > u16Pulse)
        {
          // Pulse Too Short
          m_u16ValidFrames = 0;
          Serial.print((int)i);
          Serial.print(" t<");
          Serial.println(MIN_PULSE_WIDTH);
          g_RCIFlags[i].bValid = FALSE;
        }
        else if (u16Pulse > MAX_PULSE_WIDTH)
        {
          // Pulse Too Long	
          m_u16ValidFrames = 0;
          Serial.print((int)i);
          Serial.print(" t>");
          Serial.println(MAX_PULSE_WIDTH);
          g_RCIFlags[i].bValid = FALSE;          
        }	
        else
        {
          m_u16ValidFrames++;  // TEMP - this actaully counts each valid pulse as a frame - needs refining
          g_u16Pulse[i] = u16Pulse;
          g_RCIFlags[i].bValid = TRUE;
          g_RCIFlags[i].bUpdate = TRUE;           
        }
        //Serial.print(" ");
      }
    }
  }
}


