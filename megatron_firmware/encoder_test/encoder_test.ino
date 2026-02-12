/* 

MIT License
Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab

Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

*/
#include "QuadEncoder.h"


uint32_t LeftCurPosValue;
uint32_t Left_old_position = 0;
uint32_t RightCurPosValue;
uint32_t Right_old_position = 0;

QuadEncoder LeftEnc(1, 1, 0, 0);  // Encoder on channel 1 of 4 available, 
                                    // Phase A (pin0), PhaseB(pin1), Pullups Req(0)
QuadEncoder RightEnc(2, 2, 3, 0);  // Encoder on channel 1 of 4 available, 
                                    // Phase A (pin0), PhaseB(pin1), Pullups Req(0)                                    

void setup()
{
  while(!Serial && millis() < 4000);

  /* Initialize the ENC module. */
  LeftEnc.setInitConfig();  //
  // with above settings count rev every 20 ticks
  // if myEnc1.EncConfig.revolutionCountCondition = ENABLE;
  // is not defined or set to DISABLE, the position is zeroed every
  // 20 counts, if enabled revolution counter is incremented when 
  // phaseA ahead of phaseB, and decrements from 65535 when reversed.
  LeftEnc.init();
    /* Initialize the ENC module. */
  RightEnc.setInitConfig();  //
  // with above settings count rev every 20 ticks
  // if myEnc1.EncConfig.revolutionCountCondition = ENABLE;
  // is not defined or set to DISABLE, the position is zeroed every
  // 20 counts, if enabled revolution counter is incremented when 
  // phaseA ahead of phaseB, and decrements from 65535 when reversed.
  RightEnc.init();
  
}

void loop(){
  
  /* This read operation would capture all the position counter to responding hold registers. */
  LeftCurPosValue = LeftEnc.read();
  RightCurPosValue = RightEnc.read();

  //Left
  if(LeftCurPosValue != Left_old_position){
    /* Read the position values. */
    Serial.printf("Left encoder value: %ld\r\n", LeftCurPosValue);
    Serial.println();
  }
  Left_old_position = LeftCurPosValue;

  //Right
  if(RightCurPosValue != Left_old_position){
    /* Read the position values. */
    Serial.printf("Right encoder value: %ld\r\n", RightCurPosValue);
    Serial.println();
  }
  RightCurPosValue = LeftCurPosValue;

}