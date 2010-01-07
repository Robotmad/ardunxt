// To be depricated in favour of using an RCInput channel to implement the Failsafe/MUX control as part of this code rather than reading digital inputs from the ATTiny

/****************************************************************
 ****************************************************************/
// Digital Inputs
// MUX: 
//   L = 0 Failsafe, RC control of Servo outputs
//   H = Autonomous Mode 1 or 2, NXT control of Servo outputs
// Mode: (only valid when Failsafe is NOT active)
//   L = 1
//   H = 2
//
byte RCInput_Switch(void)
{
  // Read MUX input
  if(digitalRead(4) == HIGH)
  {
    // Autonomous - read Mode input
    if(digitalRead(5) == HIGH)
    {
      return 0x02; // Mode 2
    }
    else
    {
      return 0x01; // Mode 1
    }
  }
  else
  {
    // Failsafe (either no RC signal or Tx switch in position 0)
    return 0x00; 
  }
}


void RCInput_Monitor(void)
{
  static byte m_u8LastMode;
  
  byte u8Mode = RCInput_Switch();
  
  if (m_u8LastMode != u8Mode)
  {
    Serial.print("Mode: ");
    Serial.println((int)u8Mode);
    m_u8LastMode = u8Mode;
  }
}
