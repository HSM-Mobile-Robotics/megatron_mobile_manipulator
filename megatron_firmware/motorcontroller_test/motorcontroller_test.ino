/* 

MIT License
Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab

Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

*/

//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library

#include "RoboClaw.h"

RoboClaw roboclaw(&Serial2,10000);

#define address 0x80

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(460800);
}

void loop() {
  roboclaw.ForwardM1(address,64); //start Motor1 forward at half speed
  roboclaw.ForwardM2(address,64); //start Motor2 backward at half speed
  delay(5000);

  roboclaw.BackwardM1(address,64);
  roboclaw.BackwardM2(address,64);
  delay(5000);

  roboclaw.ForwardM1(address,64); //start Motor1 forward at half speed
  roboclaw.BackwardM2(address,64); //start Motor2 backward at half speed
  delay(5000);

  roboclaw.BackwardM1(address,64);
  roboclaw.ForwardM2(address,64);
  delay(5000);

}
