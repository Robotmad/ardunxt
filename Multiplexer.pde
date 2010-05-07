// Remote Control - Servo PWM Output Multiplexer
//
// In order to provide a robust failsafe a hardware multiplexer is employed which can directly route the RC Inputs to the Servo PWM Outputs
// On the Ardupilot prototype hardware for this project there is a seperate program running on an ATTiny
// which monitors the RC Control channel and uses this to manage the multiplexer - informing us of the current state via pin 4.
//
// Alternatively we can use one of the RC inputs, which we are measuring, as a control channel and manage the state of the multiplexer
// directly ourselves by using pin 4 as an output. (To test on the ardupilot hardware connect JP6 pin 5 to pin 6 to hold the ATTiny in reset
// so that we can drive the TMISO multiplexer control signal directly.)

// Hardware defines
#define MULTIPLEXER_CTRL_PIN    (4)
#define MUX_ATMEGA_CONTROL      (1)          // High signal to/from multiplexer for ATMEGA control of Servo Output
#define MUX_RADIO_CONTROL       (0)          // Low signal to/from multiplexer for Radio Control of Servo Output


#define DFLT_MUX_CONTROL        (MUX_ATMEGA_CONTROL)  // How should we start up multiplexer
#define FAILSAFE_MUX_CTRL       (MUX_ATMEGA_CONTROL)  // How to set multiplexer in failsafe mode

// RC Input Channel and threshold for switching
#define MUX_CTRL_CHANNEL        (3)          // RC Input channel used to control multiplexer
#define MUX_PULSEWIDTH_THRESH   (1350)       // uS 
#define NUM_VALID_FRAMES_REQ    (50)


// Modes (keep in sync with Mux Control above)
#define MODE_RADIO              (0)          // Radio in control
#define MODE_ATMEGA             (1)          // ATMEGA in control
#define MODE_FAILSAFE           (2)          // No Radio signal - failsafe


// Variables
byte                             m_u8CurrentMode;  //Current Multiplexer Mode
UINT_32                          m_u32LastChangeTime;



void Init_Multiplexer(void)
{
  Serial.println("Init_Multiplexer");
#ifdef _ATTINY_IN_USE  
  // On Prototype hardware (using Ardupilot) Mux is an input
  pinMode(MULTIPLEXER_CTRL_PIN, INPUT);      // MUX input pin from ATTiny
  m_u8CurrentMode = digitalRead(MULTIPLEXER_CTRL_PIN);
#else
  // On Production hardware Mux is an output to control the multiplexer
  pinMode(MULTIPLEXER_CTRL_PIN, OUTPUT);     // Control signal for multiplexer
  digitalWrite(MULTIPLEXER_CTRL_PIN, DFLT_MUX_CONTROL);  // Default state for multiplexer
  m_u8CurrentMode = DFLT_MUX_CONTROL;
#endif
  m_u32LastChangeTime = 0;
}


// Return the current state of the Multiplexer
byte Multiplexer_State(void)
{
  return(m_u8CurrentMode);
}


void Multiplexer_Handler(void)
{
  static byte u8NewMode;
  
  byte u8Mode;
  
#ifdef _ATTINY_IN_USE
// Digital Inputs
// MUX: 
//   L = 0 Failsafe, RC control of Servo outputs
//   H = Autonomous Mode, NXT control of Servo outputs
  u8Mode =  digitalRead(MULTIPLEXER_CTRL_PIN);

  if (m_u8CurrentMode != u8Mode)
  {
    if (g_DiagnosticsFlags.bMultiplexer)
    {
      // Diagnostics for multiplexer
      Serial.print("Mode: ");
      Serial.println((int)u8Mode);
    }
    m_u8CurrentMode = u8Mode;          
  }

#else
  if (NUM_VALID_FRAMES_REQ < RCInput_ValidFrames())
  {
    // Input pulse width is already checked for validity
    if (MUX_PULSEWIDTH_THRESH > RCInput_RawCh(MUX_CTRL_CHANNEL))
    {
      u8Mode = MODE_RADIO;
    }
    else
    {
      // Long pulse width
      u8Mode = MODE_ATMEGA;
    }
  }
  else
  {
    // RC signal isn't valid - failsafe
    u8Mode = MODE_FAILSAFE;
  }

  if (u8Mode != u8NewMode)
  {
    // Reset timer for Mode selection stability
    m_u32LastChangeTime = millis();    
    u8NewMode = u8Mode;
  }
  
  if (u8Mode != m_u8CurrentMode)
  {  
    // Manage transition between states to avoid spurious pulses
    // Enforce a period of stability?
    // TODO - wait for all RCInputs to be low
    // Force restart of PWM output?
    switch (u8Mode)
    {
    case MODE_RADIO:
      // Allow instant switch to Radio Control
      digitalWrite(MULTIPLEXER_CTRL_PIN, MUX_RADIO_CONTROL);  // Drive Multiplexer control signal  
      m_u8CurrentMode = MODE_RADIO;
      if (g_DiagnosticsFlags.bMultiplexer)Serial.println("Mode: Radio");
      break;
      
    case MODE_ATMEGA:
      if (100 < (millis() - m_u32LastChangeTime))
      {
        digitalWrite(MULTIPLEXER_CTRL_PIN, MUX_ATMEGA_CONTROL);  // Drive Multiplexer control signal         
        m_u8CurrentMode = MODE_ATMEGA;
        if (g_DiagnosticsFlags.bMultiplexer)Serial.println("Mode: ArduNXT");
      }
      break;
      
    case MODE_FAILSAFE:
      if (1500 < (millis() - m_u32LastChangeTime))
      {
        // Don't switch to failsafe too quickly - i.e. allow time to recover from individual invalid RC pulses
        digitalWrite(MULTIPLEXER_CTRL_PIN, FAILSAFE_MUX_CTRL);  // Drive Multiplexer control signal         
        m_u8CurrentMode = MODE_FAILSAFE;
        if (g_DiagnosticsFlags.bMultiplexer)Serial.println("Mode: Failsafe");        
      }
      break;
    }
  }
#endif
}

