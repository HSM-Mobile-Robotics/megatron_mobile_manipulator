/*

MIT License
Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab

Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

*/

#ifndef _PID_H_
#define _PID_H_

#include <Arduino.h>

class PID {
private:
  float kp = 0.0, ki = 0.0, kd = 0.0;
  double errorIntegral = 0.0, errorDifferential = 0.0, errorPrev = 0.0;
  int maxlimit = 0, minlimit = 0;
  float velResponseLimit = 0.0, filterAlpha = 1.0, filterPrevValue = 0.0;
  unsigned long noResponseTime;
  bool coastFlag = 0;

public:
  void init(float kp, float ki, float kd, int minlimit, int maxlimit,
            float filterAlpha);
  int update(float velReq, float velActual, float dt, bool coastFlag);
};

#endif