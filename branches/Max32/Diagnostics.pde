// Performance Diagnostics Reporting
#define MIN_REPORTED_EXECUTION_TIME     (5)                   // Report instances of any execution loops which take more than 10mS to execute
                                                              // Taking much longer than this can lead to RC Input frames being missed
#define REPORTING_PERIOD                (1000)                // Report perofrmance every 1000mS

UINT_32                                  m_u32Loops;          // Loop Counter
UINT_32                                  m_u32LastLoopTime;   // Time at end of last loop execution
UINT_32                                  m_u32LastDiagTime;   // Time at last diagnostics output 

void Init_Diagnostics(void)
{
  m_u32LastLoopTime = 0;
  m_u32LastDiagTime = 0;
  m_u32Loops = 0;
}


  // Software performance monitoring
  // The more frequently we can execute the main loop the better - as there is less chance of missing anything...
void Diagnostics_Handler(void)  
{
  UINT_32  u32Now = millis();
  UINT_16  u16ExecutionTime = (UINT_16)(u32Now - m_u32LastLoopTime);
 
  // How long did the loop take to execute
  // Useful for detecting any process which can take a long time   
  if (MIN_REPORTED_EXECUTION_TIME < u16ExecutionTime)
  {
      // Report a long execution time
      Serial.print(F("Execution (mS): "));
      Serial.println(u16ExecutionTime);
  }
  m_u32LastLoopTime = u32Now;
    
  // To measure average loop excution rate we count how many loops are executed per second  
  if (REPORTING_PERIOD < (u32Now - m_u32LastDiagTime))
  {
    // Time to report number of executions
    m_u32LastDiagTime = u32Now;
    Serial.print(F("Loops: "));
    Serial.println(m_u32Loops);    
    m_u32Loops = 0;
  }
  m_u32Loops++;        // Count the number of times the loop is executed
}
