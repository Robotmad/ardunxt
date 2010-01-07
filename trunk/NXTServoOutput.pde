
// 4 channels of PWM servo control output
// 0 & 1 are high resolution (1uS)
// 2 & 3 are lower resolution (8uS)
// Update Frequency is 50Hz (20mS period) standard for normal servos
// Channels 2 & 3 can be disabled by requesting a pulse width of 0 (further work required to implement this in channels 0 & 1)

#define PWM_PERIOD		(40000U)        // PWM Output period TODO - formula to convert from uS or mS to ticks

#define MIN_PULSE_WIDTH         (1000U)  // uS
#define MAX_PULSE_WIDTH         (2000U)  // uS    

#define NUM_SERVO_CH            (4U)      // total number of servo output channels
#define NUM_HW_SERVO_CH         (2U)      // 2 hardware servo output channels

// First two servo channels have hardware support on PB1 and PB2
// 0 = PB1 = Arduino D9
// 1 = PB2 = Arduino D10

// Extra servo channels are driven by interrupt software using Timer 2
// Hence we need to use the most efficient means of contorlling the output pins - i.e. direct port manipulation.
// 2 = PB0 = Arduino D8
// 3 = PD7 = Arduino D7
#define SERVO_2_HIGH     PORTB |= 0x01  // PB0
#define SERVO_2_LOW      PORTB &= 0xFE
#define SERVO_3_HIGH     PORTD |= 0x80  // PD7
#define SERVO_3_LOW      PORTD &= 0x7F

// Extra Servo PWM
#if (NUM_HW_SERV_CH < NUM_SERVO_CH)
static byte  g_u8ExtraServoPulseWidth[NUM_SERVO_CH - NUM_HW_SERVO_CH];
#endif

/**************************************************************
 * Configuring the PWM hadware...
 ***************************************************************/
void Init_ServoOutput(void)
{   
  Serial.println("Init_ServoOutput");

  //Defining servo output pins
  digitalWrite(9,LOW);  // Channel 0
  pinMode(9,OUTPUT);
  digitalWrite(10,LOW); // Channel 1
  pinMode(10,OUTPUT);
  digitalWrite(8,LOW);  // Channel 2
  pinMode(8,OUTPUT);  
  digitalWrite(7,LOW);  // Channel 3
  pinMode(7,OUTPUT); 

  TCNT1 = 0;	          // Clear the timer

  // Force OC1A/B states to low before we enable pins as compare outputs (and while we are still in "normal" mode for Timer 1)
  TCCR1C = ((1<<FOC1A)|(1<<FOC1B));                // By faking a compare match

  // Timer 1 settings for Fast PWM to support the two hardware PWM outputs
  TCCR1A = ((1<<WGM11)|(1<<COM1B1)|(1<<COM1A1));   // Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all... 
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);        // Prescaler set to 8, that give us a resolution of 0.5us, read page 134 of data sheet

  pulse_servo_0(0);  // 0 = Off
  pulse_servo_1(0);  // 0 = Off
  pulse_servo_2(0);  // 0 = Off
  pulse_servo_3(0);  // 0 = Off

  ICR1 = PWM_PERIOD;                     // 50Hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000, 

  // Timer 1 setting to generate an interrupt which we can use to control the period of the interrupt driven extra servo outputs
  // and some of the timing fucntionality of measuring the RCInputs
  TIMSK1 |= (1 << ICIE1);                // See page 136, timer 1 interrupt mask

#if (NUM_HW_SERVO_CH < NUM_SERVO_CH)
  // Setting up Timer 2 to control the interrupt driven extra servo outputs pulse width
#if ((NUM_HW_SERVO_CH + 1) < NUM_SERVO_CH)
  // Using OCR2A and OCR2B to time Extra Servo Output Pulse Widths
  TCCR2A = _BV(WGM21) | _BV(WGM20);      // Fast PWM Mode running to TOP (0xFF) (so that it continues to run past OCR2A when OCR2B is higher
  // we catch the overflow to stop the timer again.
  TCCR2B = 0;                            // Timer not running until we are actually timing a pulse          
  TIMSK2 = _BV(OCIE2A) | _BV(OCIE2B) | _BV(TOIE2);    // Interrupt mask for counter A, counter B and Overflow
#else                                         
  // Only using OCR2A to time Extra Servo Output Pulse Width
  TCCR2A = _BV(WGM21);                   // CTC Mode running to OCR2A (so that it stops at OCR2A automatically)
  TCCR2B = _BV(CS20) | _BV(CS22);        // Prescaler 128, at 16MHz (128/16)=8, the counter will increment 1 every 8uS (Hence range is 8uS x 256 = 2048uS)
  TIMSK2 = _BV(OCIE2A);                  // Interrupt mask for counter A
#endif
#endif

  // NB sei(), to enable interrupts, is in Init_RCInputCh()
}

/*************************************************************************
 * 
 *************************************************************************/
ISR(TIMER1_CAPT_vect)//This is a timer 1 interrupt, executed every 20mS 
{
#if (NUM_HW_SERVO_CH < NUM_SERVO_CH)
  OCR2A = g_u8ExtraServoPulseWidth[0];
  // Is Servo Output 2 active (0=inactive)
  if (OCR2A)
  {
    SERVO_2_HIGH;  // Start servo output pulse active (high)   
  }
#if ((NUM_HW_SERVO_CH + 1) < NUM_SERVO_CH)  
  OCR2B = g_u8ExtraServoPulseWidth[1];
  // Is Servo Output 3 active (0=inactive)
  if (OCR2B)
  {
    SERVO_3_HIGH;
  }
#endif
  TCNT2 = 0;                             // Restarting the counter of timer 2
  TCCR2B = _BV(CS20) | _BV(CS22);        // Start Timer with prescaler 128, at 16MHz (128/16)=8, the counter will increment 1 every 8uS
#endif  

  // Code to support RCInput pulse width measurement Timer 1 overflow detection
  // We are not using a loop because we want fast code in the interrupt handler
#if (0 < NUM_PWMI)
  g_RCIFlags[0].bTimerWrap = TRUE;
#endif
#if (1 < NUM_PWMI)
  g_RCIFlags[1].bTimerWrap = TRUE;
#endif
#if (2 < NUM_PWMI)
  g_RCIFlags[2].bTimerWrap = TRUE;
#endif
#if (3 < NUM_PWMI)
  g_RCIFlags[3].bTimerWrap = TRUE;
#endif  
}

/*************************************************************************
 * 
 *************************************************************************/
#if (NUM_HW_SERVO_CH < NUM_SERVO_CH)
ISR(TIMER2_COMPA_vect, ISR_NAKED) //Interrupt of timer 2 compare A
{
  // Timer2 compare A = End Servo PWM ouput pulse
  // No need to save SREG as the only instruction we want to use ("sbi") does not modify it.
  SERVO_2_LOW;
  reti();            // We must provide the return from interrupt because of using ISR_NAKED
}
#endif

#if ((NUM_HW_SERVO_CH + 1) < NUM_SERVO_CH)
ISR(TIMER2_COMPB_vect, ISR_NAKED) //Interrupt of timer 2 compare B  (try ISR_NAKED?)
{
  // Timer2 compare B = End Servo PWM output pulse
  // No need to save SREG as the only instruction we want to use ("sbi") does not modify it.
  SERVO_3_LOW;
  reti();            // We must provide the return from interrupt because of using ISR_NAKED
}


// As we allow Timer 2 to run to TOP (so that if OCR2B is higher than OCR2A we do not reset the timer when it reaches OCR2A) 
// Thus we need to stop it running (and hence generating more compare match interrupts
ISR(TIMER2_OVF_vect, ISR_NAKED)
{
  TCCR2B = 0;        // Stop timer running until we need to time the next pulse  
  reti();            // As long as the line above compiles to a load instruction it will not affect SREG and we can use ISR_NAKED  
}
#endif

/**************************************************************
 * Function to set servo 0 pulse width
 ***************************************************************/
void pulse_servo_0(unsigned int u16PulseWidth)
{
  if (u16PulseWidth)
  {
    u16PulseWidth = constrain(u16PulseWidth, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    OCR1A = u16PulseWidth << 1;
 
    if (g_DiagnosticsFlags.bServoOutput)
    {
      Serial.print("Servo 0: ");
      Serial.println(u16PulseWidth);
    }
  }
  else
  {
    // Off
    OCR1A = 0;  // 0 would still give a very short pulse
  }
}

// Notes on trying to be able to cleanly disable PWM output
// care not to generate spikes on disabling or re-enabling
// DISABLE Don't do anything immediately - set up an interrupt for when the next pulse finsihes
//  TIMSK1 |= _BV(OCIE1A);  // Enable interrupt when next pulse has been generated
//
// ENABLE ? risk of spike?  May need to force OC1A state first by force compare (in normal mode)
//  TCCR1A |= (1<<COM1A1);   // Enable OC1A to control the pin
//
//ISR(OCIE1A)
//{
//  TCCR1A &= ((1<<WGM11)|(1<<COM1B1));   // Disconnect OC1A from the output pin
//  TIMSK1 &= byte ~_BV(OCIE1A);          // Disable this interrupt
//}

/**************************************************************
 * Function to set servo 1 pulse width
 ***************************************************************/
void pulse_servo_1(unsigned int u16PulseWidth)
{
  if (u16PulseWidth)
  {
    u16PulseWidth = constrain(u16PulseWidth, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    OCR1B = u16PulseWidth << 1;

    if (g_DiagnosticsFlags.bServoOutput)
    {
      Serial.print("Servo 1: ");
      Serial.println(u16PulseWidth);
    }
  }
  else
  {
    // Off
    OCR1B = PWM_PERIOD;  // I think this will give no pulse but a permanently high output!
  }
}


/**************************************************************
 * Functions to set extra servos pulse widths
 ***************************************************************/
void pulse_servo_2(unsigned int u16PulseWidth)
{
  if (u16PulseWidth)
  {
    u16PulseWidth = constrain(u16PulseWidth, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    g_u8ExtraServoPulseWidth[0] = u16PulseWidth >> 3;  // i.e. /8 as resolution of Timer2 is 8uS

    if (g_DiagnosticsFlags.bServoOutput)
    {
      Serial.print("Servo 2: ");
      Serial.println(u16PulseWidth);
    }
  }
  else
  {
    // Off
    g_u8ExtraServoPulseWidth[0] = 0U;
  }
}

void pulse_servo_3(unsigned int u16PulseWidth)
{
  if (u16PulseWidth)
  {
    u16PulseWidth = constrain(u16PulseWidth, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    g_u8ExtraServoPulseWidth[1] = u16PulseWidth >> 3;  // i.e. /8 as resolution of Timer2 is 8uS
    if (g_DiagnosticsFlags.bServoOutput)
    {
      Serial.print("Servo 3: ");
      Serial.println(u16PulseWidth);
    }
  }
  else
  {
    // Off
    g_u8ExtraServoPulseWidth[1] = 0U;
  }
}


// TODO high level functions to do angle to pulse conversion
// centre control, range control, reverse...




